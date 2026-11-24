/* REVERT(dp168 2026-11-13): MUST be the first lines — these feature macros
 * were only ever passed via the phantom EXTRA_BUILD_FLAGS and compiled every
 * CSD async/witness region out (lib AND its headers). Top-of-file, before
 * xua_audiohub_st.h, is the only place that catches all uses. */
#ifndef CSD_UPS_ASYNC
#define CSD_UPS_ASYNC 1
#endif
#ifndef CSD_ADC_DIAG
/* REVERT(dp185): disabled — the adc_cnt/or/sum per-word witnesses are
 * superseded by g_csd_w_portn/portz (dp181) and cost ~10 t/frame on the
 * armed wire loop, which dp184 measurement showed has zero margin.
 * adc_* INFO fields read boot values while this is 0. */
#define CSD_ADC_DIAG 0
#endif
// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
/**
 * @file xua_audiohub.xc
 * @brief XMOS USB 2.0 Audio Reference Design.  Audio Functions.
 * @author Ross Owen, XMOS Semiconductor Ltd
 *
 * This thread handles I2S and forwards samples to the SPDIF Tx core.
 * Additionally this thread handles clocking and CODEC/DAC/ADC config.
 **/

#include <syscall.h>
#include <platform.h>
#include <xs1.h>
#include <xclib.h>
#include <xs1_su.h>
#include <string.h>
#include <xassert.h>

#include "xua.h"

#include "audiohw.h"
#include "audioports.h"
#if (XUA_SPDIF_TX_EN)
#include "spdif.h"
#endif
#if (XUA_ADAT_TX_EN)
#include "adat_tx.h"
#ifndef ADAT_TX_USE_SHARED_BUFF
#error Designed for ADAT tx shared buffer mode ONLY
#endif
#endif

#if (XUA_NUM_PDM_MICS > 0)
#include "xua_pdm_mic.h"
#if (MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME != 1)
#error Only sample based interface supported between mic array and XUA
#endif
#endif

#if (AUD_TO_USB_RATIO > 1)
#include "src.h"
#endif

#include "xua_commands.h"
#include "xc_ptr.h"

#define DEBUG_UNIT XUA_AUDIOHUB
#include "debug_print.h"

#ifndef _XUA_ENABLE_I2S_TIMING_CHECK
    #define _XUA_ENABLE_I2S_TIMING_CHECK (0)
#endif

#define OUT_CHAN_COUNT (I2S_CHANS_DAC + (8*XUA_ADAT_TX_EN) + (2*XUA_SPDIF_TX_EN))
unsigned samplesOut[XUA_MAX(NUM_USB_CHAN_OUT, OUT_CHAN_COUNT)];

/* Two buffers for ADC data to allow for DAC and ADC I2S ports being offset */
#define IN_CHAN_COUNT (I2S_CHANS_ADC + XUA_NUM_PDM_MICS + (8*XUA_ADAT_RX_EN) + (2*XUA_SPDIF_RX_EN))

unsigned samplesIn[2][XUA_MAX(NUM_USB_CHAN_IN, IN_CHAN_COUNT)];

#if (XUA_ADAT_TX_EN)
extern buffered out port:32 p_adat_tx;
#endif

#if (XUA_ADAT_TX_EN)
extern clock    clk_mst_spd;
#endif

#if CODEC_MASTER
void InitPorts_slave
#else
void InitPorts_master
#endif
(buffered _XUA_CLK_DIR port:32 p_lrclk, buffered _XUA_CLK_DIR port:32 p_bclk, buffered out port:32 (&?p_i2s_dac)[I2S_WIRES_DAC],
    buffered in port:32  (&?p_i2s_adc)[I2S_WIRES_ADC]);


unsigned dsdMode = DSD_MODE_OFF;

/* Opt-in, and deliberately narrower than CSD_GEN_EN.
 *
 * This monitor is a call from HandleSampleClock -- the per-frame deadline path
 * that arms the I2S port tiles -- into application code the compiler cannot see
 * or inline. It is instrumentation, and instrumentation that changes the timing
 * of the path it measures is worthless there. Measured cost of leaving it in by
 * default: the built-in mic path delivered constant zero on both channels.
 * So ask for it by name with -DCSD_MB_I2S_MONITOR when you want the numbers. */
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER) && defined(CSD_MB_I2S_MONITOR)
/* Implemented in the application (csd_gen_mailbox.c): I2S-deadline monitor,
 * see the call site in AudioHub_MainLoop. externs rather than an include
 * because lib_xua must not depend on application headers. */
void csd_mb_note_i2s_delta(unsigned short delta_ticks);
#endif


#if defined(CSD_UPS_ASYNC)
/* Runtime USB->I2S ratio (owner directive 2026-11-08): the wire (LRCLK =
 * MCLK/128) paces the loop; one host pair is consumed every `ratio` frames.
 * Implemented in the application's C module; externs rather than an include,
 * same reason as above: lib_xua must not depend on application headers.
 * Words by value: no XC/C pointer-kind mismatch across translation units. */
int csd_ups_slot(unsigned ch, int w);
#endif

#if (DSD_CHANS_DAC != 0) && (NUM_USB_CHAN_OUT > 0)
#include "audiohub_dsd.h"
#endif

#if defined(CSD_UPS_ASYNC)
/* File-scope on purpose: AudioHwConfig and the audio loop share this core, so
 * the rate-change path hands the ratio over without a channel. 1 = stock
 * behaviour (host pair every LRCLK frame); 2/4/8 only while the CSD
 * upsampler is armed. Anything else falls back to 1. */
/* REVERT(dp168): was `static` — the app-side arm (csd_ups_block.c) wrote a
 * DIFFERENT same-named global; the lib copy stayed 1 forever, which is why
 * the runtime ratio never engaged. Non-static now links the two sides.
 * (dp167: EXTRA_BUILD_FLAGS is a phantom — the CSD_UPS_ASYNC/CSD_ADC_DIAG
 * defines below are the only way anything gated ever compiles.) */
unsigned g_csd_usb_ratio = 1;

#define CSD_MIC_PHASE_AUTO_X 0xFFFFFFFFu   /* must equal csd_upsample_hook.h */
/* 2026-11-10 PORT DECIMATOR (owner design; replaces the 11-09 latch decimator,
 * which is deleted).
 * WHY: the ADC1 port is HARDWARE paced at the wire rate -- 384 kHz at all times
 * while the upsampler is armed (owner invariant: codec runs at MCLK/128) --
 * while the USB mic endpoint runs at the host speaker rate. Something must
 * turn 384k port words into 48k mic words.
 * WHAT: count port words and present a NEW value downstream once every N words.
 * N is the wire/host ratio the board already publishes as g_csd_usb_ratio --
 * 8 at a 48 kHz host, 4 at 96 kHz, 2 at 192 kHz, 1 when host and wire match --
 * so there is no second rate source that can drift, and "update the number
 * according to the host speaker rate" is automatic.
 * The count is carried by audioToUsbRatioCounter, which the pacing block
 * already maintains and resets at each USB OUT transfer: the cycle is therefore
 * anchored to the host speaker clock, which is the only meaningful phase
 * reference (owner: without phase 0 there is no meaning to phase).
 * PHASE (MIDI DIAG:UPSAMPLing:PHASE) picks WHICH count of the N-cycle is
 * presented; auto = N-1, the word the stock last-word-standing write delivered
 * implicitly, so ratio 1 and the auto phase are value-identical to stock.
 * PER-WORD COST (owner budget -- minimum compute): one load, one compare, and
 * on the published word the store the stock code performed anyway. No call, no
 * second counter, no modulo, no division.
 * REVERT: delete this block, the two gated-store sites below, the re-arm in the
 * pacing block, and xua_csd_set/get_mic_phase.
 *
 * PLAN B -- COUNT-FREE ALTERNATIVE (owner proposal 2026-11-10; armed, not built)
 * Replace the compare-and-publish entirely with a double buffer whose publish
 * trigger IS the host sample rate, so the datapath contains no counting at all:
 *   - buffer A: written by the ADC1 port read on EVERY wire frame, paced by the
 *     port's own clock (MCLK/128 while armed). Never read by the USB side.
 *   - buffer B: the mic buffer the USB capture stream drains.
 *   - publish = copy A -> B at the USB OUT transfer boundary, the same point
 *     where readBuffNo already flips. That boundary is generated by the host's
 *     own frame clock, so it fires at the host sample rate by construction:
 *     48k times/s at 48 kHz, 96k at 96 kHz. No counter, no compare, no slot.
 * The word it presents is therefore whichever wire word was latest at the
 * boundary -- sample-and-hold, which is what the stock last-word-standing write
 * did implicitly, and value-identical at ratio 1.
 * COST OF THIS SHAPE, stated so the trade is not discovered later: the presented
 * slot is fixed by the timing of the boundary, not chosen. The phase control
 * (DIAG:UPSAMPLing:PHASE) has nothing to select, so the bit-perfect phase sweep
 * -- the eight-way check that identifies which interpolator phase carries the
 * original sample -- cannot run. Plan B can pass a fixed-phase loopback; it
 * cannot prove WHICH phase is the identity.
 * WHEN TO SWITCH: if the counting shape fails for a reason that cannot be fixed
 * in the compare itself. It has not failed: measured 2026-11-10, published words
 * = port reads / N exactly (68,882/8 per channel at a 48 kHz host, 76,794/4 at
 * 96 kHz, 102,378/2 at 192 kHz), slot = N-1 in each case.
 * REVERT (Plan B, if ever built): delete the two gated stores, add the A->B copy
 * at the transfer boundary. */
static unsigned g_csd_mic_phase = CSD_MIC_PHASE_AUTO_X;  /* control plane, MIDI */
static unsigned g_csd_dec_slot  = 0u;  /* count presented, armed for next cycle */
static unsigned g_csd_dec_pub   = 0u;  /* witness: words presented to downstream */

/* 2026-11-09 raw ADC1 witness (assistant), for the ratio>1 acquisition fault.
 * WHY: three excitations (unique random words, isolated impulses, and a DC
 * constant on BOTH channels) produced ZERO occurrences of the driven value in
 * the 48 kHz mic stream while the device reported armed at ratio 8
 * (DIAG:UPSAMPLing:INFO? in_hz=48000 out_hz=384000 ratio=8), and ALSA's own
 * /proc/asound/card3/pcm0c/sub0/hw_params confirmed the capture endpoint
 * negotiated 48 kHz. So the disagreement is not in the decimator or the
 * buffer/flip logic (both audited sound) -- it is in what the port read hands
 * over. WHAT: per channel, count of IN instructions executed, bitwise OR of
 * every raw word, and running sum. A count that advances with an OR of 0 means
 * the port returns silence (bus/clock problem); a count that does not advance
 * means the read is being gated out; words matching the stimulus mean the
 * corruption is downstream of the read. HOW TO REVERT: delete this block, the
 * two #ifdef CSD_ADC_DIAG blocks at the fill sites, and the six tail fields in
 * DIAG:UPSAMPLing:INFO?. Cost when the flag is undefined: nothing -- the
 * counters stay 0 and the fields read 0. */
/* REVERT(dp168 2026-11-13): the dp167 lesson — EXTRA_BUILD_FLAGS is a
 * phantom (never consumed by the build; verified twice via symbol table).
 * Everything armed-loop related is therefore UNCONDITIONAL here and is
 * runtime-gated on g_csd_usb_ratio. Stock single-rate behaviour must be
 * byte-identical (ratio 1 => same branch selections as the stock lines). */
unsigned g_csd_c_pass = 0, g_csd_c_gateblk = 0, g_csd_c_gtrue = 0,
         g_csd_c_gfalse = 0, g_csd_c_tail = 0, g_csd_c_reentry = 0;
unsigned g_csd_f_armed_n = 0, g_csd_f_wait_n = 0, g_csd_f_skip_n = 0,
         g_csd_f_wt = 0;
/* REVERT(dp168): pacing witness globals re-added after the dp168 recovery
 * (csd_diag_scpi externs them; values: wire_* mirror the floor latch, the
 * it_/fl_ split-timing family stays 0 — CSD_IT_SPLIT never existed outside
 * the phantom flags). */
unsigned g_csd_wire_ticks_val = 0, g_csd_wire_rem_val = 0;
unsigned g_csd_it_adc_ticks = 0, g_csd_it_pre_ticks = 0, g_csd_fl_ticks = 0;
unsigned g_csd_it_n = 0, g_csd_fl_wait = 0;
/* REVERT(dp168): lib-side witnesses re-added (lost in the dp168 baseline
 * revert; all unconditional — flags are dead). */
unsigned g_csd_gate_true = 0, g_csd_gate_false = 0, g_csd_ratio_live = 0;
unsigned g_csd_tt_ticks = 0, g_csd_tt_n = 0, g_csd_tt_a = 0;
unsigned g_csd_xfer_tok = 0, g_csd_xfer_else = 0;
unsigned g_csd_adc_ctr_seen = 0, g_csd_adc_fc_seen = 0;
extern unsigned csd_ups_diag_out_hz(void);
extern unsigned csd_ups_get_want(void);
void csd_ps_thread(void);            /* csd_ps_thread.xc (REVERT dp161) */
int  csd_ps_claim(unsigned ch, int w); /* csd_ps_ring.c   (REVERT dp161) */
/* REVERT(dp187): the R-half claim reads these directly (see R site). */
extern unsigned g_csd_ps_active;
extern int      g_csd_ps_latch1;
/* REVERT(dp189): mailbox (csd_ps_ring.c). The L site seqlock-reads these;
 * asm ldw/stw names them by symbol, which needs file-global linkage. */
extern unsigned g_csd_ps_mbox_seq;
/* REVERT(dp209): queue claim words (csd_ps_ring.c, de-stated) + cursors. */
extern int g_csd_ps_w0[8];
extern int g_csd_ps_w1[8];
extern unsigned g_csd_ps_consumed;
extern unsigned g_csd_ps_starve;
extern unsigned g_csd_ps_fresh;      /* REVERT(dp196): fresh clock */
/* REVERT(dp196): attribution witnesses (defined csd_ps_ring.c, written the
 * L-site accept path, read via DIAG:UPSAMPLing:PACE?). */
extern unsigned g_csd_wire_l0, g_csd_wire_seen, g_csd_wire_s1,
                g_csd_wire_s2, g_csd_wire_n, g_csd_wire_tile;
extern int      g_csd_ps_mbox_w0, g_csd_ps_mbox_w1;
/* REVERT(dp171): per-iteration span at the floor site (ticks, 10ns). */
unsigned g_csd_span_min = 0xFFFFFFFFu, g_csd_span_max = 0, g_csd_span_last = 0;
unsigned g_csd_floor_n = 0, g_csd_f_dmax = 0;  /* REVERT(dp172) */
unsigned g_csd_claim_pace_n = 0;                           /* REVERT(dp177) */
unsigned g_csd_claim_wnext = 0, g_csd_claim_now = 0, g_csd_claim_stp = 0; /* REVERT(dp177) */
unsigned g_csd_f_add_n = 0, g_csd_f_wnext = 0, g_csd_f_now2 = 0;
unsigned g_csd_f_wnext2 = 0;   /* REVERT(dp177) */
unsigned g_csd_dn_min = 0xFFFFFFFFu, g_csd_dn_max = 0;   /* REVERT(dp174) */
unsigned g_csd_resync_n = 0;
/* REVERT(s15-diag): receive_command wake attribution -- which c_aud command
 * unwinds the audiohub loop (and thus re-runs AudioHwConfig) each time. */
unsigned g_csd_cmd_tot = 0, g_csd_cmd_sr = 0, g_csd_cmd_start = 0;
unsigned g_csd_cmd_stop = 0, g_csd_cmd_unh = 0, g_csd_cmd_last = 0;
unsigned g_csd_adc_cnt[2] = {0, 0};
unsigned g_csd_adc_or[2] = {0, 0};
/* 2026-11-09 divider witness (assistant), for the four-way config question
 * (ES9219Q 384k / ADC1 384k / USB out 48k / USB in 48k). WHY: the wire clock is
 * decided ONLY by these values at "divide = mClk / (curSamFreq * numBits)",
 * where curSamFreq is the host's SET_CUR value; csd_AudioHwConfig runs later,
 * so the board's family-rate force cannot reach the divider. WHAT: latch the
 * exact triple used to program the ports, plus what the board tried to force.
 * HOW TO REVERT: delete these globals, the two record lines, and the
 * DIAG:UPSAMPLing:DIVider? command. Runs once per rate change -- not the hot
 * path. */
unsigned g_csd_div_curSamFreq = 0, g_csd_div_mClk = 0, g_csd_div_divide = 0;
unsigned g_csd_div_numBits = 0, g_csd_div_dsdMode = 0;
unsigned g_csd_forced_samFreq = 0, g_csd_forced_mClk = 0, g_csd_forced_seq = 0;
/* 2026-11-09 fabric-rate override (assistant). WHY: measured DIVider? in the
 * armed state was curSamFreq=48000, mClk=49152000, divide=16, numBits=64 ->
 * BCLK 3.072 MHz -> LRCLK 48 kHz, while the board forced the codec to 384 kHz
 * (forced_samFreq=384000, seq=1). The ADC witness agreed: ~48.9 k IN
 * instructions per second, not 384 k. So the owner's required configuration
 * (ES9219Q 384k / ADC1 384k / USB out 48k / USB in 48k) was structurally
 * unmeetable: divide at :1192 and ConfigAudioPortsWrapper at :1262 both read
 * curSamFreq (the host's SET_CUR), and AudioHwConfig -- where the board applies
 * its family force -- runs afterwards at :1289, so it can only reprogram the
 * codec script and the MCLK PLL. WHAT: the board publishes the wire rate it
 * forced; after AudioHwConfig the audiohub re-runs the port configuration from
 * that rate, leaving curSamFreq (USB pacing, feedback, endpoint sizing) at the
 * host rate. HOW TO REVERT: delete this setter, the re-configuration block
 * below it, and the xua_csd_set_fabric_force() call in clearsounddac_board.xc.
 * At ratio==1 the published value is 0 and nothing here executes. */
/* Prototypes for the branch-local decision below. XC refuses function
 * declarations at block scope ("cannot declare a function in local scope"), so
 * they live here. Definitions are in the app (src/dsp/csd_ups_channel.c) and
 * run in the caller's context. */
unsigned csd_ups_get_want(void);
unsigned csd_ups_is_enabled(void);
unsigned csd_ups_expected_out_hz(unsigned hz);
/* DoSampleTransfer stall statistics, filled only by the CSD_XFER_TIMING block
 * at the transfer point. Timer ticks are 10 ns on XS3, so us = ticks/100.
 * DEFINED UNCONDITIONALLY so the SCPI tail (a C translation unit that does not
 * receive CSD_EXTRA_FLAGS) always links and reads three zeros when the probe is
 * off. REVERT: delete these and the two timing guards. */
unsigned g_csd_xfer_n = 0, g_csd_xfer_ticks = 0, g_csd_xfer_max = 0;
/* Per-channel-operation stall totals (10 ns ticks) -- see
 * CompleteSampleTransferTimed. REVERT with it. */
unsigned g_csd_xt_ct = 0, g_csd_xt_in = 0, g_csd_xt_out = 0;
unsigned g_csd_xt_ct_n = 0, g_csd_xt_in_n = 0, g_csd_xt_out_n = 0;
unsigned g_csd_xt_ubm = 0, g_csd_xt_ubm_n = 0, g_csd_xt_ubm_max = 0;
#ifdef CSD_UPS_ASYNC
/* One-pair prefetch pipeline for the upsampling regime (see the CSD_UPS_ASYNC
 * block in the pacing code). REVERT: delete these two and that block. */
unsigned g_csd_prefetch = 0, g_csd_prefetch_hits = 0;
#endif
unsigned g_csd_fabric_force_hz = 0;
/* What THIS block last programmed the ports with. Sentinel-initialised so the
 * first configuration after boot always (re-)applies, and so a rate change back
 * to the host rate is not mistaken for "already correct". */
unsigned g_csd_applied_hz = 0xFFFFFFFFu;
unsigned g_csd_applied_divide = 0xFFFFFFFFu;
void xua_csd_set_fabric_force(unsigned hz) { g_csd_fabric_force_hz = hz; }

void xua_csd_record_forced(unsigned samFreq, unsigned mClk)
{
    g_csd_forced_samFreq = samFreq;
    g_csd_forced_mClk = mClk;
    g_csd_forced_seq++;
}

unsigned g_csd_adc_sum[2] = {0, 0};   /* definitions unconditional so DIAG can
                                       * always report them; only the INCREMENTS
                                       * are compile-gated, so a build without
                                       * -DCSD_ADC_DIAG reads all-zero and pays
                                       * nothing in the per-slot path. */

/* LRCLK bit-slot timing witnesses (2026-11-10, 384 kHz EQ fault). Writer:
 * audio() task in HandleSampleClock (gated by CSD_EQ_DIAG_WITNESS); reader:
 * SCPI diag thread through DIAG:UPSAMPLing:INFO? tail. ts_min is seeded at
 * 0xFFFF so the first real delta seeds it. REVERT: delete these and the
 * guarded block in HandleSampleClock + the INFO? tail fields. */
unsigned g_csd_lrclk_emit = 0;
unsigned g_csd_lrclk_ts_min = 0xFFFFu;
unsigned g_csd_lrclk_ts_max = 0;

/* REVERT(dp181): mic zero-birth witnesses. WHY: at ratio-2 armed the mic
 * stream carries ~40% exact-zero frames (runs of ~48, valid runs ~44);
 * dp180 proved the echo tap injects none of it (hold-last change moved
 * nothing). These counters localize the ZERO BIRTH across the three
 * word-handling points of the stock USB-IN path: (1) raw ADC port IN
 * word, (2) port-decimator store, (3) USB ship (outuint). Per point and
 * channel: total words and exact-zero words. Deltas over a controlled
 * armed window answer: port reads zero (codec/clock phase), or ships
 * zero from a nonzero store (stale buffer shipped = staging/parity
 * fault on the reader side). Two increments per word worst-case
 * (~768k/s): ~5 MIPS on a core that measured 1000 available; the same
 * shape of witness (CSD_ADC_DIAG) ran armed before. Unconditional per
 * the dp167 phantom-flag rule; INFO? tail fields 121..131. */
unsigned g_csd_w_portn[2] = {0,0}, g_csd_w_portz[2] = {0,0};
unsigned g_csd_w_pubn[2]  = {0,0}, g_csd_w_pubz[2] = {0,0};
unsigned g_csd_w_shipn[2] = {0,0}, g_csd_w_shipz[2] = {0,0};
unsigned g_csd_w_exch = 0;      /* exchanges that shipped (non-CT path) */
unsigned g_csd_w_iter = 0;      /* REVERT(dp203): unconditional while-loop-top
                                 * counter — distinguishes 'reads skipped'
                                 * (iter==exch>portn) from 'gate ran twice
                                 * per read' (iter==portn<exch) on ratio-1
                                 * legs. One stw/iter, exch precedent. */
unsigned g_csd_w_dsdn = 0;      /* REVERT(dp203): iterations that took the
                                 * DSD/DOP dispatch branch (no ADC reads) —
                                 * the only source-level skip path. */
unsigned g_csd_w_portu[2] = {0,0}; /* REVERT(dp203): unconditional raw-read
                                 * witnesses (bit5-gate removal control). */
/* REVERT(dp183): runtime attribution probe mask (defined in the app's
 * csd_diag_scpi.c; set/query via DEBUG:CSD:PROBE). bit0 = skip the two
 * csd_ups_slot pop calls; bit1 = skip the top-of-loop wire floor;
 * bit4 = accumulate per-segment timer witnesses into g_csd_seg[]
 * (DEBUG:CSD:SEG?). */
extern unsigned g_csd_probe_mask;
/* REVERT(dp185): segment timing witnesses [0..5]: L-port-read pass,
 * R-port-read pass, DAC-L out+claim, DAC-R out+claim, floor block,
 * exchange block (gate-true frames only). Ticks accumulated; seg_n counts
 * frames. Only advanced when probe mask bit4 set (hot path: 1 test). */
unsigned g_csd_seg[8] = {0,0,0,0,0,0,0,0};
unsigned g_csd_seg_n = 0;
/* REVERT(dp199-attr): whole-iteration timer (bit4-gated) + the sub-slices
 * that were missing from the dp185 map: DAC-L out and DAC-R out (seg[4]
 * repurposed from the floor — the floor is off by default and its own
 * block already self-times via f_wait/f_skip), while-top->seg0a (seg[6])
 * and the loop tail after the exchange gate (seg[7]). With these eight +
 * g_csd_loop_ticks the ENTIRE wire iteration is accounted; any residue is
 * per-op wake latency. */
unsigned g_csd_loop_ticks = 0;
unsigned g_csd_seg_b[6] = {0,0,0,0,0,0}; /* REVERT(dp199-attr): [0] DAC-L out,
                                       * [1] DAC-R out, [2] while-top..ADC-L,
                                       * [3] DAC-R-out-end..loop-bottom,
                                       * [4] tailA..exchange-gate (dsd chk +
                                       *     TDM bookkeeping),
                                       * [5] gate-false else path (per frame
                                       *     it runs) */
static unsigned csd_gf_n = 0;          /* gate-false path sample count */
/* REVERT(dp199-attr): LRCLK emit-interval witnesses per half-frame parity
 * (bit5). port-ts domain, stock perfect = 32. Accessors for csd_diag_scpi. */
unsigned g_csd_lr_min[2] = {0xFFFFu, 0xFFFFu};
unsigned g_csd_lr_max[2] = {0, 0};
unsigned g_csd_lr_sum[2] = {0, 0};
unsigned g_csd_lr_n[2]   = {0, 0};
void csd_lr_wit_get(unsigned *mn, unsigned *mx, unsigned *sm, unsigned *n)
{ for (int i = 0; i < 2; i++) { mn[i]=g_csd_lr_min[i]; mx[i]=g_csd_lr_max[i];
                                 sm[i]=g_csd_lr_sum[i]; n[i]=g_csd_lr_n[i]; } }
void csd_lr_wit_clear(void)
{ for (int i = 0; i < 2; i++) { g_csd_lr_min[i]=0xFFFFu; g_csd_lr_max[i]=0;
                                g_csd_lr_sum[i]=0; g_csd_lr_n[i]=0; } }
/* REVERT(dp199-attr): accessors for the SEG?/SEG:CLEAR surface (C file). */
unsigned csd_gf_n_get(void) { return csd_gf_n; }
void     csd_gf_n_clear(void) { csd_gf_n = 0; }

/* Arm which count of the N-cycle the port decimator presents. Runs on the
 * control plane only -- ratio hand-overs and PHASE writes, never per word.
 * auto (or a choice at/above the last count) presents the LAST count, which is
 * the word the stock last-word-standing write already delivered, so the default
 * configuration is value-identical to stock. REVERT: delete with the decimator. */
static void xua_csd_decim_rearm(void)
{
    const unsigned n = g_csd_usb_ratio;
    const unsigned q = g_csd_mic_phase;
    g_csd_dec_slot = (q == CSD_MIC_PHASE_AUTO_X || q >= (n - 1u)) ? (n - 1u) : q;
}

void xua_csd_set_usb_ratio(unsigned r)
{
    g_csd_usb_ratio = (r == 8u || r == 4u || r == 2u) ? r : 1u;
    /* REVERT(dp191): close the prefetch-wedge. The two-phase exchange
     * (CSD_UPS_ASYNC) opens a StartSampleTransfer token one transfer frame
     * early; if a ratio>1 stream ENDS in that window the token's reply is
     * never consumed and g_csd_prefetch stays 1 into the next stream. The
     * first exchange frame then runs CompleteSampleTransfer's 2x inuint
     * against a reply the restarted decouple will never send: the wire
     * loop wedged (measured dp190, second armed stream: 1,984 it/s,
     * hostZ 1.0000). Decouple resets its stream FSM at stream start, so
     * dropping the flag here re-synchronises both sides; an orphan token
     * already queued by decouple only yields a one-frame-stale reply —
     * exactly the lag the prefetch design already runs with. */
    g_csd_prefetch = 0u;
    /* REVERT(dp202): disarmed hand-over takes the DAC path back to stock.
     * g_csd_ps_active is refreshed ONLY by the producer's armed /16 pass
     * (csd_ps_ring.c); after an armed session then a ratio-1 stream, the
     * producer parks without clearing, the wire claim sites keep the
     * mailbox arm and ship stale csd_l0/csd_l1 -> a disarmed 384k leg
     * following an armed leg played silence (measured mic RMS 40k vs the
     * 2.97M ideal). Clear here -- control plane, wire-thread writer, once
     * per stream start; NEVER on the armed path (r>1 does not clear), so
     * the armed steady state stays byte-identical to the dp201-certified
     * image. */
    if (g_csd_usb_ratio == 1u)
    { asm volatile("stw %0, dp[g_csd_ps_active]" :: "r"(0u) : "memory"); }
    /* 2026-11-10 port decimator: re-anchor at every ratio hand-over (the board
     * re-issues this at each AudioHwConfig -- stream start and every rate
     * change). WHY: N follows the host speaker rate, so the presented count must
     * be re-derived from the new N -- a slot chosen under the old N can sit
     * outside the new cycle and present nothing at all. At ratio 1 this yields
     * slot 0, which every frame matches: the stock path, by construction.
     * REVERT: delete this call. */
    xua_csd_decim_rearm();
}
/* 2026-11-09: read side, for DIAG:UPSAMPLing:INFO?. Read-only: it observes the
 * same file-scope the frame loop already reads, and changes nothing. NOTE: like
 * every static in this file it belongs to the tile that links lib_xua, so the
 * value is the audiohub's own -- which is the one that paces the wire.
 * REVERT: delete this function and its header row. */
unsigned xua_csd_get_usb_ratio(void)
{
    return g_csd_usb_ratio;
}
/* 2026-11-10 port decimator observation (DIAG surface). WHY the audiohub owns
 * these: N and the count live here, next to the pacing block that drives them,
 * so the diagnostic cannot disagree with the datapath. Read-only.
 * REVERT: delete with the decimator. */
unsigned xua_csd_get_decim_slot(void) { return g_csd_dec_slot; }
unsigned xua_csd_get_decim_pub(void)  { return g_csd_dec_pub; }
void xua_csd_set_mic_phase(unsigned p)
{
    g_csd_mic_phase = (p < 8u) ? p : CSD_MIC_PHASE_AUTO_X;
    /* Control plane (MIDI/SCPI task, never the frame loop): arm the presented
     * count for the next cycle. REVERT: delete this call. */
    xua_csd_decim_rearm();
}
unsigned xua_csd_get_mic_phase(void)
{
    return g_csd_mic_phase;
}
#endif

#if (XUA_ADAT_TX_EN)
#include "audiohub_adat.h"
#endif
#include "xua_audiohub_st.h"

static inline int HandleSampleClock(int frameCount, buffered _XUA_CLK_DIR port:32 p_lrclk, int first_frame)
{
#if CODEC_MASTER
    unsigned syncError = 0;
    unsigned lrval = 0;
    const unsigned lrval_mask = (0xffffffff << (32 - XUA_I2S_N_BITS));

    if(XUA_I2S_N_BITS != 32)
    {
        asm volatile("in %0, res[%1]":"=r"(lrval):"r"(p_lrclk):"memory");
        set_port_shift_count(p_lrclk, XUA_I2S_N_BITS);
    }
    else
    {
        p_lrclk :> lrval;
    }

    if(XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
    {
        /* Only check for the rising edge of frame sync being in the right place because falling edge timing not specified */
        if (frameCount == 1)
        {
            lrval &= 0xc0000000;                 // Mask off last two (MSB) frame clock bits which are the most recently sampled
            syncError += (lrval != 0x80000000);  // We need MSB = 1 and MSB-1 = 0 to signify rising edge
        }
        else
        {
            /* We do not check this part of the frame because TDM frame sync falling egde timing
             * is not defined. We only care about rising edge which is checked in first half of frame */
        }
    }
    else
    {
        if(XUA_I2S_N_BITS == 32)
        {
            if(frameCount == 0)
                syncError = (lrval != 0x80000000);
            else
                syncError = (lrval != 0x7FFFFFFF);
        }
        else
        {
            if(frameCount == 0)
                syncError = ((lrval & lrval_mask) != 0x80000000);
            else
                syncError = ((lrval | (~lrval_mask)) != 0x7FFFFFFF);
        }
    }

    return syncError;

#else
    static unsigned short port_ts, prev_port_ts;
    unsigned clkVal;
    if(XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
    {
        if(frameCount == (I2S_CHANS_PER_FRAME-1))
            clkVal = 0x80000000;
        else
            clkVal = 0x00000000;
    }
    else
    {
        if(frameCount == 0)
            clkVal = 0x80000000;
        else
            clkVal = 0x7fffffff;
    }

    if(XUA_I2S_N_BITS == 32)
    {
        p_lrclk <: clkVal;
        asm volatile(" getts %0, res[%1]" : "=r" (port_ts) : "r" (p_lrclk));
        /* REVERT(dp192 TRIED, REVERTED dp193): gating this getts behind
         * probe bit5 was measured (passT, 384k, dp192): the loopback tone
         * degraded from peak 1001.1 Hz/87 dB to peak-bin 0 Hz garbage.
         * The getts apparently paces something on the p_lrclk port path
         * (timestamp sync semantics on the buffered port); do NOT gate it
         * again without re-running both gate legs. */
    }
    else
    {
        partout(p_lrclk, XUA_I2S_N_BITS, clkVal >> (32 - XUA_I2S_N_BITS));
    }

    if(!first_frame)
    {
        unsigned short diff = port_ts - prev_port_ts;
#ifdef CSD_EQ_DIAG_WITNESS
        /* LRCLK bit-slot timing witness (2026-11-10, 384 kHz EQ fault).
         * LRCLK is generated by THIS loop (master mode): if the loop ever
         * runs late, the gap between two timed p_lrclk emits stretches and
         * the codec sees a jittered frame clock. Count the emits and record
         * the min/max observed inter-emit delta so a budget overrun is
         * visible from the host. Writer: audio() task; reader: SCPI diag
         * thread (informational, same rules as g_csd_adc_*). REVERT: delete
         * this block and the g_csd_lrclk_* definitions above. */
        /* REVERT(dp204): ONE mask snapshot per emit (the parity block used
         * to load g_csd_probe_mask itself), and the emit counter + ts
         * sentinels moved behind bit5 with it: at PROBE 0 the whole block
         * is one shared load + test; before, the emit++ RMW and the two
         * sentinel compares ran on every half-frame of the zero-margin
         * ratio-1 loop (2 emits/frame). Fields lrclk_emit/ts_min/ts_max
         * stay at boot values unless DEBUG:CSD:PROBE 32 is set BEFORE the
         * stream (same contract as adc_* under CSD_ADC_DIAG 0). */
        const unsigned wit_m = g_csd_probe_mask;
        if (wit_m & 32u) {
            g_csd_lrclk_emit++;
            unsigned p = (frameCount == 0) ? 0u : 1u;
            if (diff < g_csd_lr_min[p]) g_csd_lr_min[p] = diff;
            if (diff > g_csd_lr_max[p]) g_csd_lr_max[p] = diff;
            g_csd_lr_sum[p] += diff;
            g_csd_lr_n[p]++;
        }
#endif
#if _XUA_ENABLE_I2S_TIMING_CHECK
        asm volatile("ecallf %0":: "r" (diff == 32));
#endif
        //xassert((diff == 32)); // compiling with asserts enabled has a separate set of problems, even without this check (sw_usb_audio issue 340)
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER) && defined(CSD_MB_I2S_MONITOR)
        /* I2S-deadline monitor -- the digital twin of the case-A question.
         *
         * concurrent_fork.py closed the fork on the FIRST stream after load:
         * the mic hears +25.2 dB over the no-stream floor while the firmware
         * records every DAC-bound word of that same stream as SILENCE (five
         * nodes, peak -240 dBFS, self-checked against a real tone). So the
         * sample VALUES are provably clean and something on the wire itself
         * is wrong. If mixer1 misses the sample period, this port timestamp
         * delta drifts from its 32-tick budget: the loop then re-emits a
         * frame, or the BCLK/LRCLK edges stretch, and the codec synthesises
         * periodic garbage from a clean-but-late bitstream. This is exactly
         * what XMOS' own -DENABLE_I2S_TIMING_CHECK / ecallf catches ("to fix
         * I2S timing violation causing intermittent test failures",
         * sw_usb_audio changelog) -- ecallf is fatal here, so we count.
         * Writer: audio() task. Read: csd_mb_get_i2s_* over SCPI. */
        csd_mb_note_i2s_delta(diff);
#endif
    }
    prev_port_ts = port_ts;

    return 0;
#endif

}

#pragma unsafe arrays
unsigned static AudioHub_MainLoop(chanend ?c_aud, chanend ?c_spd_out
#if (XUA_ADAT_TX_EN)
    , chanend c_adat_out
    , unsigned adatSmuxMode
#endif
    , unsigned divide, unsigned curSamFreq
#if (XUA_SPDIF_RX_EN || XUA_ADAT_RX_EN)
    , chanend c_dig_rx
#endif
#if (XUA_NUM_PDM_MICS > 0)
    , chanend c_pdm_pcm
#endif
    , buffered _XUA_CLK_DIR port:32 ?p_lrclk,
    buffered _XUA_CLK_DIR port:32 ?p_bclk,
    buffered out port:32 (&?p_i2s_dac)[I2S_WIRES_DAC],
    buffered in port:32  (&?p_i2s_adc)[I2S_WIRES_ADC]
)
{
    /* Since DAC and ADC buffered ports off by one sample we buffer previous ADC frame */
    unsigned readBuffNo = 0;
    unsigned index;

#if (DSD_CHANS_DAC != 0)
    unsigned dsdMarker = DSD_MARKER_2;    /* This alternates between DSD_MARKER_1 and DSD_MARKER_2 */
    int dsdCount = 0;
    int everyOther = 1;
    unsigned dsdSample_l = 0x96960000;
    unsigned dsdSample_r = 0x96960000;
#endif
    unsigned underflowWord = 0;

#if (XUA_ADAT_TX_EN)
    adatCounter = 0;
#endif

#if(DSD_CHANS_DAC != 0)
    if(dsdMode == DSD_MODE_DOP)
    {
        underflowWord = 0xFA969600;
    }
    else if(dsdMode == DSD_MODE_NATIVE)
    {
        underflowWord = 0x96969696;
    }
#endif

    unsigned audioToUsbRatioCounter = 0;
#if (XUA_NUM_PDM_MICS > 0)
    unsigned audioToMicsRatioCounter = 0;
#endif

#if (AUD_TO_USB_RATIO > 1)
    union i2sInDs3
    {
        long long doubleWordAlignmentEnsured;
        int32_t delayLine[I2S_DOWNSAMPLE_CHANS_IN][SRC_FF3V_FIR_NUM_PHASES][SRC_FF3V_FIR_TAPS_PER_PHASE];
    } i2sInDs3;
    memset(&i2sInDs3.delayLine, 0, sizeof i2sInDs3.delayLine);
    int64_t i2sInDs3Sum[I2S_DOWNSAMPLE_CHANS_IN];

    union i2sOutUs3
    {
        long long doubleWordAlignmentEnsured;
        int32_t delayLine[I2S_CHANS_DAC][SRC_FF3V_FIR_TAPS_PER_PHASE];
    } i2sOutUs3;
    memset(&i2sOutUs3.delayLine, 0, sizeof i2sOutUs3.delayLine);
#endif /* (AUD_TO_USB_RATIO > 1) */

    UserBufferManagementInit(curSamFreq);

    /* Get initial samples for first I2S output */
    unsigned command = DoSampleTransfer(c_aud, readBuffNo, underflowWord);

    // Reinitialise user state before entering the main loop
    UserBufferManagementInit(curSamFreq);

#if (XUA_ADAT_TX_EN)
    unsafe{
    //TransferAdatTxSamples(c_adat_out, samplesOut, adatSmuxMode, 0);
    volatile unsigned * unsafe samplePtr = &samplesOut[ADAT_TX_INDEX];
    outuint(c_adat_out, (unsigned) samplePtr);
    }
#endif
    if(command != XUA_AUDCTL_NO_COMMAND)
    {
        return command;
    }

    /* REVERT(dp168): wire-cadence floor state (see wait at the frame tail) */
    timer wireTmr; unsigned wireNext = 0, wireTicks = 0, wireRem = 0, wireAcc = 0;
    unsigned wireStal = 0u;  /* REVERT(dp187): staleness-divider for the
                              * resync check (was a per-frame timer read) */
    /* REVERT(dp189): mailbox pair registers + last-seq for the wire read
     * (csd_ps_ring.c produce_pair publishes; single writer, this reader). */
    unsigned csd_mbox_last = 0u, csd_l0 = 0, csd_l1 = 0;

    timer tseg;   /* REVERT(dp185): segment timing witness timer (mask bit4) */
    unsigned wireF = 1u;   /* REVERT(dp184): cached wire rate for the carry */
    /* REVERT(dp177): per-claim wire pacer — the exchange bursts multiple
     * frames per outer iteration, so the outer floor cannot pace claims:
     * measured claims 554k/s vs floor 443k/s vs wire 384k/s; mic_stale
     * 554k/s = every sample stale. Same wireTmr: both waits are
     * sequential on this single thread. */
    unsigned claimNext = 0, claimStep = 0;
    g_csd_c_reentry++; /* REVERT(dp168) */

    /* Main Audio I/O loop */
    while (1)
    {
        unsigned syncError = 0;
        unsigned frameCount = 0;
        unsigned skip_ts_check = 1; // Skip I2S port timestamp check for the very first frame
            const unsigned pmask_wire = g_csd_probe_mask; /* REVERT(dp202): one load per wire iteration; the per-site mask tests used to be individual cross-region loads on a zero-margin loop (dp184) -- passT leg paid ~10% wire rate for them. Snapshot per iteration is probe-lag <= 1 frame. */

        if ((I2S_CHANS_DAC > 0 || I2S_CHANS_ADC > 0))
        {
#if CODEC_MASTER
            InitPorts_slave(p_lrclk, p_bclk, p_i2s_dac, p_i2s_adc);
#else
            InitPorts_master(p_lrclk, p_bclk, p_i2s_dac, p_i2s_adc);
#endif
        }

        /* Note we always expect syncError to be 0 when we are master */
        while(!syncError)
        {
            if (pmask_wire & 32u) { g_csd_w_iter++; } /* REVERT(dp203): bit5-gated
             * (unconditional form cost 4 t/iter on the zero-margin ratio-1 loop,
             * measured 379.7k/s vs 384k stride). */
            /* REVERT(dp199-attr): iteration accounting anchors (bit4). */
            unsigned loopA = 0, tailA = 0;
            if (pmask_wire & 16u) tseg :> loopA;
            /* REVERT(dp200-emit-first): armed (ratio>1) leg uses the
             * half-frame pipelined emit schedule (see the emit blocks);
             * ratio-1 (passT path) keeps the stock order. Read per
             * iteration: the ratio is set at runtime on stream start. */
            const unsigned csd_armed = (g_csd_usb_ratio > 1u);
            /* REVERT(dp172): THE wire schedule — top of loop, once per
             * iteration, before any work. Prior floors sat at the frame
             * tail: the exchange overshoot then always re-expired the next
             * deadline before it could be observed (f_wait stayed 0 across
             * dp166-171). Hard wait-first form; deep-stall resync only. */
            /* REVERT(dp190): the floor is OFF by default — PROBE mask bit1
             * (DEBUG:CSD:PROBE 2) now ENABLES it for attribution runs
             * (inverted from dp183's skip-bit). WHY: with the producer
             * mailbox (dp189) off the wire thread, the floor block is a
             * measured 65 t/frame (dp189 rate_probe: mask0 306.8k vs mask2
             * 383.5k = the port-paced rate, hostZ 0.0007). The loop needs
             * no software pacer: this loop EMITS the I2S clocks (master)
             * and every frame ends in buffered-port traffic (p_lrclk <:
             * per half-frame, p_i2s_dac <: per channel) whose hardware
             * drain clock caps the emission at the wire rate — the
             * 383,454/s mask2 rate IS the port clock. dp168 added the
             * floor because the armed loop then free-ran at 221-245k —
             * that was the claim/exchange stall, since moved off-thread;
             * the floor's wait branch never fired (always behind), so it
             * was pure cost. The ratio-1 pass leg has always run this way
             * (floor is ratio>1 gated) and is gate-CLEAN. f_add_n moved
             * out of the block; the block's own counters stay inside it. */
            /* REVERT(dp205): the floor is ON by default for armed legs —
             * PROBE mask bit1 (DEBUG:CSD:PROBE 2) now OPTS OUT. WHY: dp190's
             * "the buffered port drain caps the loop at the wire rate" claim
             * is FALSE on this build. Measured (image ff386d75, 48k armed,
             * PROBE 0, 8 s window): wire_n 495,758/s (+29% over the 384,000
             * nominal), popsL 452,575/s, and DAC slots are 24-bit
             * (1.95 us/frame of port drain = 512k/s ceiling) so the port
             * never blocks the loop at 384k. The free-run rate is the
             * producer/consumer side effect the acceptance forbids:
             * pops/s must equal R x host_rate. The accumulator below is the
             * only rate authority on the wire; enabling it costs one timer
             * op on an early loop (the loop runs 201 t/frame vs a 260 t
             * stride, so it WILL absorb). */
            if (csd_armed && !(pmask_wire & 2u))
            {
                if (wireTicks == 0u)
                {
                    unsigned wf = csd_ups_diag_out_hz();
                    /* REVERT(dp209): re-latch retry. The single-shot latch
                     * reads out_hz at the FIRST armed iteration — before the
                     * rate hand-over has published it (0 or stale), and it
                     * never retried: wireTicks stayed 0, the whole floor was
                     * skipped every iteration (f_add_n witness frozen at 0
                     * across all campaigns — the tell we mis-read), and the
                     * loop free-ran at machine rate (PROBE-32 iters 679k/s
                     * at r2). The exchange gate then drained host pairs at
                     * the iteration rate against the host refill: periodic
                     * duplicate host frames = the comb. Retry every 8192
                     * iterations until a plausible rate latches (3-4 latches
                     * = one tile-timer read amortized ~0). */
                    static unsigned wrl_n = 0u;
                    if (++wrl_n >= 8192u) { wrl_n = 0u;   /* poll gate */ }
                    else if (wireTicks == 0u && !(wf >= 44100u && wf <= 768000u))
                    { /* keep waiting; retry */ }
                    else if (wf >= 44100u && wf <= 768000u)
                    {
                        wireTicks = XS1_TIMER_HZ / wf;
                        wireRem   = XS1_TIMER_HZ % wf;
                        wireF     = wf;             /* REVERT(dp184): cached
                                                     * carry modulus (was a
                                                     * per-frame cross-file
                                                     * csd_ups_diag_out_hz) */
                        wireTmr :> wireNext;
                        g_csd_f_armed_n++;
                        g_csd_wire_ticks_val = wireTicks;
                    }
                }
                if (wireTicks >= 1000000u) { wireTicks = 0u; }  /* bad latch */
                if (wireTicks != 0u)
                {
                    /* REVERT(dp184): PURE deadline accumulator (was dp178
                     * wait-then-anchor). WHY: anchoring wireNext to the
                     * OBSERVED time (t1 after the wait, or t0 on the skip
                     * branch) charges per-iteration compute+wake latency to
                     * the schedule forever, so the loop self-paced at its
                     * machine rate (measured 262.8k it/s vs 384k wire, dn
                     * mostly positive). With a bare '+= stride' grid an
                     * EARLY loop is paced exactly at the wire rate (the
                     * wait absorbs); the deep-stall resync below bounds
                     * the catch-up when the loop arrives far behind.
                     * Witness arithmetic (dn min/max, span min/max) and the
                     * per-frame f_* stores were pruned with it — they cost
                     * ~40 t/frame on the critical path (dp183 probe: mask2
                     * vs mask0). */
                    g_csd_floor_n++;
                    unsigned seg4a; if (pmask_wire & 16u) tseg :> seg4a;
                    /* REVERT(dp187): the per-frame 'wireTmr :> t0' + 3-way
                     * branch is replaced by the one wait the schedule
                     * actually uses. WHY: dp186 attribution (PROBE mask
                     * bit1 vs 0, rate_probe) priced this block at ~35 t of
                     * the 326 t/frame loop against a 260 t stride; while
                     * compute-bound the deadline is ALWAYS already past, so
                     * the read bought nothing but the wait/skip witness
                     * counts and the deep-stall guard. 'when timerafter' on
                     * an expired deadline completes immediately: one timer
                     * op paces when early, costs nothing when late. The
                     * staleness/resync check (and the wait/skip counts,
                     * now 1-in-256 samples) run every 256th frame — a deep
                     * stall then free-runs at most 256 frames before the
                     * deadline re-anchors, bounded. */
                    wireTmr when timerafter(wireNext) :> void;
                    { wireAcc += wireRem;
                      if (wireAcc >= wireF) { wireNext++; wireAcc -= wireF; } }
                    /* REVERT(dp209): deadline-miss re-anchor (the csd_ps_thread
                     * dp209 fix, same defect here). The pure accumulator has
                     * no catch-up: once wireNext slips into the past (any deep
                     * stall > 10 strides between the 256-frame resync polls,
                     * or accumulated work>jitter drift), timerafter is
                     * trivially satisfied forever and the loop FREE-RUNS at
                     * machine rate (measured PROBE-32: wire_iters 679,007/s
                     * vs the 384,000 it must run — the exchange gate then
                     * drains host pairs at 339k/s against a 192k/s refill:
                     * 50 % duplicate host frames = the comb, exactly the
                     * quantified -14 dBFS signature). Re-anchor one stride
                     * ahead when >1 stride behind; hysteresis keeps the
                     * dp185 early-pass behaviour intact. */
                    {   unsigned w_now; wireTmr :> w_now;
                        if ((int)(w_now - wireNext) > (int)wireTicks)
                        { wireNext = w_now + wireTicks; wireAcc = 0u;
                          g_csd_resync_n++; }
                    }
                    wireNext += wireTicks;
                    if (++wireStal >= 256u)
                    {
                        wireStal = 0u;
                        unsigned t0; wireTmr :> t0;
                        if ((int)(t0 - wireNext) > (int)(10u * wireTicks))
                        { g_csd_resync_n++; wireNext = t0; }
                        else if ((int)(t0 - wireNext) < 0)
                        { if (pmask_wire & 32u) g_csd_f_wait_n++; }
                        else { if (pmask_wire & 32u) g_csd_f_skip_n++; }
                    }
                    if (pmask_wire & 16u)
                    { unsigned seg4b; tseg :> seg4b; g_csd_seg[4] += seg4b - seg4a; }
                }
            }
            /* REVERT(dp190): loop-rate witness moved OUT of the optional
             * floor block (rate_probe field 130; counts wire iterations
             * at ratio>1 whether the floor is on or off).
             * REVERT(dp199): gated behind probe bit5 (DEBUG:CSD:PROBE 32) —
             * a pure witness RMW (global load+add+store) on EVERY wire
             * frame; the PACE? window witnesses (p_dwl/p_tdl) give the
             * same rate attribution without it, so the hot path pays only
             * the mask test. Read f_add_n only in PROBE-32 legs. */
            if (g_csd_usb_ratio > 1u && (pmask_wire & 32u)) { g_csd_f_add_n++; }
#if (DSD_CHANS_DAC != 0) && (NUM_USB_CHAN_OUT > 0)
            if(dsdMode == DSD_MODE_NATIVE)
            { g_csd_w_dsdn++; DoDsdNative(samplesOut, dsdSample_l, dsdSample_r, divide); }
            else if(dsdMode == DSD_MODE_DOP)
            { g_csd_w_dsdn++; DoDsdDop(everyOther, samplesOut, dsdSample_l, dsdSample_r, divide); }
            else
#endif
            {
#if (I2S_CHANS_ADC != 0)
#if (AUD_TO_USB_RATIO > 1)
                if (0 == audioToUsbRatioCounter)
                {
                    memset(&i2sInDs3Sum, 0, sizeof i2sInDs3Sum);
                }
#endif /* (AUD_TO_USB_RATIO > 1) */
                /* REVERT(dp200-emit-first): emit-first for the ARMED legs.
                 * Witness (per-parity LRCLK emit intervals, SEG? fields
                 * 18-25, PROBE 48, armed 192k leg, flashed 0dd1ec9c):
                 *   parity0 (L-slot) min=0 max=103 mean=34.2
                 *   parity1 (R-slot) min=0 max=104 mean=47.1 (port-ts,
                 *   perfect=32). The whole cadence slip sits in the
                 *   LRCLK0->LRCLK1 interval: the LRCLK1 edge is gated
                 *   BEHIND the blocking ADC-R read in the stock order
                 *   (ADC-L, emit0, DACL, ADC-R, emit1, DACR), so any
                 *   input-side delay moves a CLOCK EDGE on the wire.
                 * The armed DAC words come from the producer mailbox --
                 * no input-read dependency -- so the armed path pre-arms
                 * the pair and queues all four port ops (LRCLK0, DACL,
                 * LRCLK1, DACR) back-to-back at the top of the iteration,
                 * then consumes the port-buffered input words while the
                 * tail (exchange/UBM) runs. Ratio-1 (the CLEAN passT
                 * path) keeps the stock order, untouched. */
                if (csd_armed)
                {
                    /* REVERT(dp200): half-frame pipelined emit. ACCEPT-time
                     * witnesses (build ce470: back-to-back queueing gave
                     * HC0->HC1 accept interval 48.5 port-ts while only
                     * ~20 t of code sits between them => the p_lrclk port
                     * serializes accepts at drain granularity: the LRCLK1
                     * write cannot enter the port until LRCLK0's 32-bit
                     * shift is nearly done, so it always leaves the pin
                     * one half-word late). Interleave instead: queue
                     * LRCLK0+DACL now, consume the L input word (which
                     * returns at ~the LRCLK0 drain end), then queue
                     * LRCLK1+DACR exactly when the port is free.
                     * The words are LAST iteration's pop (pop moved below
                     * the ADC-R read, dp200): the accept path now carries
                     * nothing but the port ops -- one pair of words is in
                     * flight, the hold-last mailbox semantics already ran
                     * that way (producer leads by one pair). */
                    syncError += HandleSampleClock(0, p_lrclk, skip_ts_check);
                    unsigned sb4a; if (pmask_wire & 16u) tseg :> sb4a;
                    if(XUA_I2S_N_BITS == 32)
                        p_i2s_dac[0] <: bitrev(csd_l0);
                    else
                        partout(p_i2s_dac[0], XUA_I2S_N_BITS, bitrev(csd_l0));
                    if (pmask_wire & 16u)
                    { unsigned sb; tseg :> sb; g_csd_seg_b[0] += sb - sb4a; }
                }
                /* REVERT(dp200): pop moved below the ADC-R read -- the dead
                 * anchor block that lived here is deleted; the pair is read
                 * at the loop bottom and the emits above use LAST frame's
                 * pair (hold-last one-word-in-flight, matching the
                 * producer's one-pair lead). */
                /* Input previous L sample into L in buffer */
                unsigned seg0a; if (pmask_wire & 16u) tseg :> seg0a;
                if (pmask_wire & 16u)
                { unsigned sb; tseg :> sb; g_csd_seg_b[2] += sb - loopA; }
                index = 0;
                /* First input (i.e. frameCount == 0) we read last ADC channel of previous frame.. */
                /* REVERT(dp159 2026-11-13): frameCount never exceeds 1
                 * (I2S_CHANS_PER_FRAME==2), so the stock ternary always
                 * staged into the buffer being uploaded NOW; the next-up
                 * buffer kept boot zeros. Publish to the staging buffer. */
                unsigned buffIndex = !readBuffNo;

#pragma loop unroll
                /* First time around we get channel 7 of TDM8 */
                for(int i = 0; i < I2S_CHANS_ADC; i+=I2S_CHANS_PER_FRAME)
                {
                    // p_i2s_adc[index++] :> sample;
                    // Manual IN instruction since compiler generates an extra setc per IN (bug #15256)
                    unsigned sample;
                    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p_i2s_adc[index]));
                    if (pmask_wire & 32u) { g_csd_w_portu[0]++; } /* REVERT(dp203) */

                    sample = bitrev(sample);
                    if(XUA_I2S_N_BITS != 32)
                    {
                        set_port_shift_count(p_i2s_adc[index], XUA_I2S_N_BITS);
                        sample <<= (32 - XUA_I2S_N_BITS);
                    }
                    index++;

                    int chanIndex = ((frameCount-2) & (I2S_CHANS_PER_FRAME-1)) + i; // channels 0, 2, 4.. on each line.
                    /* REVERT(dp181): raw-word zero-birth witness (see decl).
                     * REVERT(dp186): counter stores gated behind probe mask
                     * bit5 (DEBUG:CSD:PROBE 32) — dp185 SEG timing showed
                     * the per-word witness stores are a first-order share
                     * of the armed wire budget. */
                    if ((pmask_wire & 32u) && (unsigned)chanIndex < 2u) {
                        g_csd_w_portn[chanIndex]++;
                        if (sample == 0u) g_csd_w_portz[chanIndex]++;
                    }
#if CSD_ADC_DIAG /* REVERT(dp204): was #ifdef — `#define CSD_ADC_DIAG 0`
                  * above does NOT undefine it, so the dp185 disable never
                  * took effect and these 3 RMWs/word (6/frame) ran on the
                  * zero-margin ratio-1 wire loop (measured adc_cnt
                  * advancing 384k/s with the switch 'off'). */
                    if ((unsigned)chanIndex < 2u) {   /* raw witness, see decl */
                        g_csd_adc_cnt[chanIndex]++;
                        g_csd_adc_or[chanIndex] |= sample;
                        g_csd_adc_sum[chanIndex] += sample;
                    }
#endif

#if (AUD_TO_USB_RATIO > 1)
                    if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
                    {
                        samplesIn[buffIndex][chanIndex] =
                            src_ds3_voice_add_final_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
                    else
                    {
                        i2sInDs3Sum[chanIndex] =
                            src_ds3_voice_add_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
#else
  #ifdef CSD_UPS_ASYNC
                    /* PORT DECIMATOR: present one word per N counts, N = the
                     * wire/host ratio (see g_csd_dec_slot). Non-selected words
                     * are dropped at the port. The published word then sits in
                     * the buffer until the transfer ships it -- the same
                     * lifetime the stock write had. At ratio 1 the slot is 0
                     * and audioToUsbRatioCounter is always 0: byte-identical
                     * to the stock store below. */
                    if (audioToUsbRatioCounter == g_csd_dec_slot)
  #endif
                    {
                        samplesIn[buffIndex][chanIndex] = sample;
                        /* REVERT(dp181): store-point zero-birth witness.
                         * REVERT(dp186): gated behind probe mask bit5. */
                        if ((pmask_wire & 32u) && (unsigned)chanIndex < 2u) {
                            g_csd_w_pubn[chanIndex]++;
                            if (sample == 0u) g_csd_w_pubz[chanIndex]++;
                        }
#ifdef CSD_UPS_ASYNC
                        if (pmask_wire & 32u) { g_csd_dec_pub++; } /* REVERT(dp191) */
#endif
                    }
#endif /* (AUD_TO_USB_RATIO > 1) */
                }
#endif

#if (I2S_CHANS_ADC != 0 || I2S_CHANS_DAC != 0)
                if (!csd_armed) /* REVERT(dp200-emit-first): armed emits at top */
                    syncError += HandleSampleClock(frameCount, p_lrclk, skip_ts_check);
#endif
                if (pmask_wire & 16u)
                { unsigned seg0b; tseg :> seg0b; g_csd_seg[0] += seg0b - seg0a; }
                if (csd_armed)
                {
                    /* REVERT(dp200): LRCLK1+DACR queued at the half-frame
                     * boundary (see the split-emit note above). */
                    syncError += HandleSampleClock(1, p_lrclk, skip_ts_check);
                    unsigned sb5a; if (pmask_wire & 16u) tseg :> sb5a;
                    /* One wire carries both halves (I2S_WIRES_DAC==1): the
                     * stock L/R loops each restart index at 0, i.e. both
                     * words go to p_i2s_dac[0]. */
                    if(XUA_I2S_N_BITS == 32)
                        p_i2s_dac[0] <: bitrev(csd_l1);
                    else
                        partout(p_i2s_dac[0], XUA_I2S_N_BITS, bitrev(csd_l1));
                    if (pmask_wire & 16u)
                    { unsigned sb; tseg :> sb; g_csd_seg_b[1] += sb - sb5a; }
                }

#pragma xta endpoint "i2s_output_l"

#if (I2S_CHANS_DAC != 0)
                if (!csd_armed) /* REVERT(dp200-emit-first) */
                {
                index = 0;
#pragma loop unroll
                /* Output "even" channel to DAC (i.e. left) */
                for(int i = 0; i < I2S_CHANS_DAC; i+=I2S_CHANS_PER_FRAME)
                {
#if (AUD_TO_USB_RATIO > 1)
                    if (0 == audioToUsbRatioCounter)
                    {
                        samplesOut[frameCount+i] = src_us3_voice_input_sample(i2sOutUs3.delayLine[i],
                                                                              src_ff3v_fir_coefs[2],
                                                                              samplesOut[frameCount+i]);
                    }
                    else /* audioToUsbRatioCounter == 1 or 2 */
                    {
                        samplesOut[frameCount+i] = src_us3_voice_get_next_sample(i2sOutUs3.delayLine[i],
                                                                                 src_ff3v_fir_coefs[2-audioToUsbRatioCounter]);
                    }
#endif /* (AUD_TO_USB_RATIO > 1) */
#ifdef CSD_UPS_ASYNC
                    /* REVERT(dp183): g_csd_probe_mask bit0 short-circuits the
                     * pop calls (attribution probe, csd_diag_scpi.c).
                     * REVERT(dp184): claim restored (dp161 machinery) — the
                     * dp183 probe proved the in-loop pop (78 t/frame) is
                     * what kept the wire loop 120 t over its 260 t budget;
                     * the producer thread pays it outside the frame. */
                    if (g_csd_usb_ratio > 1u && !(pmask_wire & 1u)) {
                        /* Wire-rate slot (owner directive 2026-11-08): the
                         * word for this slot comes from the upsampler's
                         * band-limited queue, not from the host pair sitting
                         * in samplesOut. csd_ps_claim returns the input word
                         * untouched when the path is not active, so a
                         * disarmed stream keeps stock behaviour. ch =
                         * frameCount + i is the wire channel (0 = L, 1 = R),
                         * the same identity the bitrev write below uses. */
                        unsigned seg2a; if (pmask_wire & 16u) tseg :> seg2a;
                        /* REVERT(dp189): csd_ps_claim(0u, w) replaced by an
                         * inline seqlock read of the producer mailbox — the
                         * wire thread no longer calls into csd_ps_ring.c at
                         * all (wall proof in csd_ps_ring.c produce_pair).
                         * s1/s2 bracket the word reads; s1 odd (producer in
                         * its publish window) or s1 != s2 (pair advanced
                         * mid-read) retries, capped at 3 then accepted.
                         * s2 == last seq = producer has not published since
                         * the previous frame -> honest repeat + starve
                         * witness, same semantics the claim gave. */
                        if (g_csd_ps_active) {
                            g_csd_wire_s1++; /* dp209: L-region liveness witness */
                            /* REVERT(dp199b): ONE seq read. The dp189
                             * bracket form (two seq ldw + compare + retry
                             * test per frame) existed to catch preemption
                             * mid-read — impossible here: the producer
                             * (csd_ps_thread) runs in audiohub's INTERNAL
                             * par, same core, cooperative (see its header),
                             * and this block contains no sync op, so the
                             * wire task is never interrupted between the
                             * seq read and the word loads. Even seq => the
                             * pair is complete and stable (producer stores
                             * words before the even bump; a mid-publish
                             * read sees ODD -> bounded retry; a producer
                             * spin is not a yield). Retains: 3-retry,
                             * honest repeat, bit5-gated starve. */
                            unsigned s1 = 1u, try_n = 0u;
                            for (;;) {
                                asm volatile("ldw %0, dp[g_csd_ps_mbox_seq]"
                                             : "=r"(s1));
                                if ((s1 & 1u) == 0u) {
                                    csd_l0 = (unsigned)g_csd_ps_mbox_w0;
                                    csd_l1 = (unsigned)g_csd_ps_mbox_w1;
                                    break;
                                }
                                if (++try_n >= 3u) break;
                            }
                            /* REVERT(dp196): the producer's pacing clock —
                             * ONE C increment per wire frame (3 memops, no
                             * asm barriers). Producer paces passes == this
                             * rate (csd_ps_thread.xc). Rejected forms:
                             * consume-SEQ cursors mis-counted through
                             * seqlock races (.63/.64/.65); the word-
                             * identity 'fresh' clock deadlocked the rate
                             * loop at the clamp whenever ring-repeats made
                             * published words equal (.71: pinned 290 t
                             * despite passes 344.8k > wire 307.0k). */
                            g_csd_wire_n++;
                            /* REVERT(dp198): seq-identity fresh witness --
                             * this frame's accepted pair MOVED (seq
                             * advanced) vs the dp196 word-identity form
                             * that deadlocked on equal published words.
                             * Diagnostic only; the dp198 controller paces
                             * on g_csd_wire_n itself.
                             * REVERT(dp199): fresh and starve folded into
                             * ONE comparison (was two compares of the same
                             * pair plus an unconditional last-store); the
                             * starve arm keeps its bit5 gate. */
                            if (s1 != csd_mbox_last && !(s1 & 1u)) {
                                g_csd_ps_fresh++;
                                csd_mbox_last = s1;
                            } else if (pmask_wire & 32u) {
                                unsigned sv;
                                asm volatile("ldw %0, dp[g_csd_ps_starve]"
                                             : "=r"(sv));
                                sv++;
                                asm volatile("stw %0, dp[g_csd_ps_starve]"
                                             :: "r"(sv));
                            }
                            samplesOut[frameCount + i] = csd_l0;
                        } else {
                            samplesOut[frameCount + i] =
                                (unsigned)csd_ups_slot(0u, (int)samplesOut[frameCount + i]);
                        }
                        if (pmask_wire & 16u)
                        { unsigned seg2b; tseg :> seg2b; g_csd_seg[2] += seg2b - seg2a; }
                    }
#endif /* CSD_UPS_ASYNC */
                    /* REVERT(dp199-attr): DAC-L out slice (bit4). */
                    unsigned sb4a; if (pmask_wire & 16u) tseg :> sb4a;
                    if(XUA_I2S_N_BITS == 32)
                        p_i2s_dac[index++] <: bitrev(samplesOut[frameCount +i]);
                    else
                        partout(p_i2s_dac[index++], XUA_I2S_N_BITS, bitrev(samplesOut[frameCount +i]));
                    if (pmask_wire & 16u)
                    { unsigned sb; tseg :> sb; g_csd_seg_b[0] += sb - sb4a; }
                }
                } /* REVERT(dp200-emit-first): !csd_armed DAC-L block end */
#endif // (I2S_CHANS_DAC != 0)

            if(frameCount == 0)
            {
#if (XUA_ADAT_TX_EN)
                TransferAdatTxSamples(c_adat_out, samplesOut, adatSmuxMode, 1);
#endif
#if (XUA_SPDIF_TX_EN) && (NUM_USB_CHAN_OUT > 0)
                outuint(c_spd_out, samplesOut[SPDIF_TX_INDEX]);  /* Forward samples to S/PDIF Tx thread */
                outuint(c_spd_out, samplesOut[SPDIF_TX_INDEX + 1]);
#endif

#if (XUA_SPDIF_RX_EN || XUA_ADAT_RX_EN)
                /* Sync with clockgen */
                inuint(c_dig_rx);

                /* Note, digi-data we just store in samplesIn[readBuffNo] - we only double buffer the I2S input data */
#endif
#if (XUA_SPDIF_RX_EN)
                asm("ldw %0, dp[g_digData]"  :"=r"(samplesIn[readBuffNo][SPDIF_RX_INDEX + 0]));
                asm("ldw %0, dp[g_digData+4]":"=r"(samplesIn[readBuffNo][SPDIF_RX_INDEX + 1]));
#endif
#if (XUA_ADAT_RX_EN)
                asm("ldw %0, dp[g_digData+8]" :"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX]));
                asm("ldw %0, dp[g_digData+12]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 1]));
                asm("ldw %0, dp[g_digData+16]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 2]));
                asm("ldw %0, dp[g_digData+20]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 3]));
                asm("ldw %0, dp[g_digData+24]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 4]));
                asm("ldw %0, dp[g_digData+28]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 5]));
                asm("ldw %0, dp[g_digData+32]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 6]));
                asm("ldw %0, dp[g_digData+36]":"=r"(samplesIn[readBuffNo][ADAT_RX_INDEX + 7]));
#endif

#if (XUA_SPDIF_RX_EN || XUA_ADAT_RX_EN)
                /* Request digital data (with prefill) */
                outuint(c_dig_rx, 0);
#endif

#if (XUA_NUM_PDM_MICS > 0)
                if ((AUD_TO_MICS_RATIO - 1) == audioToMicsRatioCounter)
                unsafe {
                    chanend_t c_m2a = (chanend_t)c_pdm_pcm;
                    int32_t *mic_samps_base_addr = (int32_t*)&samplesIn[readBuffNo][XUA_PDM_MIC_INDEX];
                    ma_frame_rx(mic_samps_base_addr, c_m2a, MIC_ARRAY_CONFIG_SAMPLES_PER_FRAME, MIC_ARRAY_CONFIG_MIC_COUNT);
                    xua_user_pdm_process(mic_samps_base_addr);
                    audioToMicsRatioCounter = 0;
                }
                else
                {
                    ++audioToMicsRatioCounter;
                }
#endif
            }

           frameCount++;

#if (I2S_CHANS_ADC != 0)
                unsigned seg1a; if (pmask_wire & 16u) tseg :> seg1a;
                index = 0;
                /* Channels 0, 2, 4.. on each line */
#pragma loop unroll
                for(int i = 0; i < I2S_CHANS_ADC; i += I2S_CHANS_PER_FRAME)
                {
                    /* Manual IN instruction since compiler generates an extra setc per IN (bug #15256) */
                    unsigned sample;
                    asm volatile("in %0, res[%1]" : "=r"(sample)  : "r"(p_i2s_adc[index]));
                    if (pmask_wire & 32u) { g_csd_w_portu[1]++; } /* REVERT(dp203) */
                    /* REVERT(dp191): per-frame debug-witness RMWs behind
                     * probe bit5 (DEBUG:CSD:PROBE 32). */
                    if (pmask_wire & 32u) {
                        g_csd_adc_ctr_seen |= 1u << (audioToUsbRatioCounter & 7u);
                        g_csd_adc_fc_seen  |= 1u << (frameCount & 7u);
                    }
                    sample = bitrev(sample);
                    if(XUA_I2S_N_BITS != 32)
                    {
                        set_port_shift_count(p_i2s_adc[index], XUA_I2S_N_BITS);
                        sample <<= (32 - XUA_I2S_N_BITS);
                    }
                    index++;

                    int chanIndex = ((frameCount-2)&(I2S_CHANS_PER_FRAME-1))+i; // channels 1, 3, 5.. on each line.
                    /* REVERT(dp181): raw-word zero-birth witness (see decl).
                     * REVERT(dp186): counter stores gated behind probe mask
                     * bit5 (DEBUG:CSD:PROBE 32) — dp185 SEG timing showed
                     * the per-word witness stores are a first-order share
                     * of the armed wire budget. */
                    if ((pmask_wire & 32u) && (unsigned)chanIndex < 2u) {
                        g_csd_w_portn[chanIndex]++;
                        if (sample == 0u) g_csd_w_portz[chanIndex]++;
                    }
#if CSD_ADC_DIAG /* REVERT(dp204): was #ifdef — see the sibling site's note */
                    if ((unsigned)chanIndex < 2u) {   /* raw witness, see decl */
                        g_csd_adc_cnt[chanIndex]++;
                        g_csd_adc_or[chanIndex] |= sample;
                        g_csd_adc_sum[chanIndex] += sample;
                    }
#endif
#if (AUD_TO_USB_RATIO > 1 && !I2S_DOWNSAMPLE_MONO_IN)
                    if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
                    {
                        samplesIn[buffIndex][chanIndex] =
                            src_ds3_voice_add_final_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
                    else
                    {
                        i2sInDs3Sum[chanIndex] =
                            src_ds3_voice_add_sample(
                                i2sInDs3Sum[chanIndex],
                                i2sInDs3.delayLine[chanIndex][audioToUsbRatioCounter],
                                src_ff3v_fir_coefs[audioToUsbRatioCounter],
                                sample);
                    }
#else
  #ifdef CSD_UPS_ASYNC
                    /* PORT DECIMATOR: present one word per N counts, N = the
                     * wire/host ratio (see g_csd_dec_slot). Non-selected words
                     * are dropped at the port. The published word then sits in
                     * the buffer until the transfer ships it -- the same
                     * lifetime the stock write had. At ratio 1 the slot is 0
                     * and audioToUsbRatioCounter is always 0: byte-identical
                     * to the stock store below. */
                    if (audioToUsbRatioCounter == g_csd_dec_slot)
  #endif
                    {
                        samplesIn[buffIndex][chanIndex] = sample;
                        /* REVERT(dp181): store-point zero-birth witness.
                         * REVERT(dp186): gated behind probe mask bit5. */
                        if ((pmask_wire & 32u) && (unsigned)chanIndex < 2u) {
                            g_csd_w_pubn[chanIndex]++;
                            if (sample == 0u) g_csd_w_pubz[chanIndex]++;
                        }
#ifdef CSD_UPS_ASYNC
                        if (pmask_wire & 32u) { g_csd_dec_pub++; } /* REVERT(dp191) */
#endif
                    }
#endif /* (AUD_TO_USB_RATIO > 1) && !I2S_DOWNSAMPLE_MONO_IN */
                }
#endif /* I2S_CHANS_ADC != 0) */

#if (I2S_CHANS_ADC != 0 || I2S_CHANS_DAC != 0)
                if (!csd_armed) /* REVERT(dp200-emit-first): armed emits at top */
                    syncError += HandleSampleClock(frameCount, p_lrclk, skip_ts_check);
#endif
                if (pmask_wire & 16u)
                { unsigned seg1b; tseg :> seg1b; g_csd_seg[1] += seg1b - seg1a; }

                /* REVERT(dp200): producer mailbox pop for NEXT frame's
                 * emits, taken here (after both input reads) so the
                 * accept path above carries nothing but port ops. The
                 * pop-L body as ever; the guarded stock DAC-R loop's own
                 * pop-R site is armed-skipped (csd_l1 is filled here). */
                if (csd_armed && !(pmask_wire & 1u))
                {
                    unsigned seg2a; if (pmask_wire & 16u) tseg :> seg2a;
                    if (g_csd_ps_active) {
                        g_csd_wire_s2++; /* dp209: R-region liveness witness */
                        /* REVERT(dp209b): claim EVERY iteration — the
                         * iteration is the frame (dp189 rate_probe: mask0
                         * == mask2 == 383.5k, the port-paced rate: the
                         * buffered LRCLK/DAC OUTs block at drain depth and
                         * ARE the pacer; the dp172 floor is off by default
                         * and was never the authority). The dp209 parity
                         * gate (claim every 2nd accept) therefore dropped
                         * a pair per frame: every second frame emitted the
                         * HELD pair — a hard 2:1 duplicate-frame stream,
                         * the exact comb generator (margin unchanged at
                         * -5.9 dB with fresh==wire because BOTH counted
                         * the halved claim beat). One claim, one frame,
                         * one pair — full stop. */
                        static unsigned csd_wn_parity = 0u; (void)csd_wn_parity;
                        {
                            g_csd_wire_n++;
                            unsigned want, seq;
                            asm volatile("ldw %0, dp[g_csd_ps_consumed]" : "=r"(want));
                            asm volatile("ldw %0, dp[g_csd_ps_seq]" : "=r"(seq));
                            if (seq != want) {
                                unsigned i = want & 7u;
                                csd_l0 = (unsigned)g_csd_ps_w0[i];
                                csd_l1 = (unsigned)g_csd_ps_w1[i];
                                unsigned nw = want + 1u;
                                asm volatile("stw %0, dp[g_csd_ps_consumed]" :: "r"(nw));
                                g_csd_ps_fresh++;   /* PACE: claims/s */
                            } else if (pmask_wire & 32u) {
                                unsigned sv;
                                asm volatile("ldw %0, dp[g_csd_ps_starve]"
                                             : "=r"(sv));
                                sv++;
                                asm volatile("stw %0, dp[g_csd_ps_starve]"
                                             :: "r"(sv));
                            }
                        }
                        samplesOut[0] = csd_l0;   /* keep samplesOut honest
                                                   * for diagnostics, but the
                                                   * emits read csd_l0/l1:
                                                   * the exchange's inuint
                                                   * rewrites samplesOut[0/1]
                                                   * later THIS iteration */
                        samplesOut[1] = csd_l1;
                    } else {
                        csd_l0 = (unsigned)csd_ups_slot(0u, (int)samplesOut[0]);
                        csd_l1 = (unsigned)csd_ups_slot(1u, (int)samplesOut[1]);
                    }
                    if (pmask_wire & 16u)
                    { unsigned seg2b; tseg :> seg2b; g_csd_seg[2] += seg2b - seg2a; }
                }

                index = 0;
#if (I2S_CHANS_DAC != 0)
                if (!csd_armed) /* REVERT(dp200-emit-first) */
                {
                /* Output "odd" channel to DAC (i.e. right) */
#pragma loop unroll
                for(int i = 0; i < I2S_CHANS_DAC; i+=I2S_CHANS_PER_FRAME)
                {
#if (AUD_TO_USB_RATIO > 1)
                    if (audioToUsbRatioCounter == 0)
                    {
                        samplesOut[frameCount+i] = src_us3_voice_input_sample(i2sOutUs3.delayLine[i],
                                                                              src_ff3v_fir_coefs[2],
                                                                              samplesOut[frameCount+i]);
                    }
                    else
                    { /* audioToUsbRatioCounter is 1 or 2 */
                        samplesOut[frameCount+i] = src_us3_voice_get_next_sample(i2sOutUs3.delayLine[i],
                                                                                 src_ff3v_fir_coefs[2-audioToUsbRatioCounter]);
                    }
#endif /* (AUD_TO_USB_RATIO > 1) */
#ifdef CSD_UPS_ASYNC
                    /* REVERT(dp183): g_csd_probe_mask bit0 (see above site).
                     * REVERT(dp184): claim restored (see above site). */
                    if (g_csd_usb_ratio > 1u && !(pmask_wire & 1u)) {
                        /* Wire-rate slot, RIGHT channel (owner directive
                         * 2026-11-08; the pop was left-channel-only, so the
                         * R ring filled and never drained -- popsR 0,
                         * occH_R at ring size, right acoustic output dead).
                         * Channel identity, re-derived 2026-11-10 and the
                         * reason a blind copy of the even-loop line once
                         * regressed the board: THIS loop runs after
                         * frameCount++ (see :722), so frameCount == 1 and
                         * I2S_CHANS_DAC=2 / I2S_CHANS_PER_FRAME=2 give the
                         * single iteration i == 0. Hence (frameCount + i)
                         * evaluates to exactly 1 -- the same index the
                         * bitrev write below reads for the R wire slot.
                         * csd_ps_claim returns w untouched when the path is
                         * not active, so a disarmed stream is stock. */
                        unsigned seg3a; if (pmask_wire & 16u) tseg :> seg3a;
                        /* REVERT(dp187): was csd_ps_claim(1u, w) — a cross-TU
                         * call whose live body is 'return s_latch1' plus the
                         * inactive fallback. dp186 bit0-vs-0 attribution put
                         * ~30 t/frame on the two claim calls; the R half is
                         * now two global loads the same thread's L claim
                         * wrote earlier THIS iteration (csd_ps_ring.c:
                         * g_csd_ps_active, g_csd_ps_latch1). Inactive path
                         * keeps the stock csd_ups_slot passthrough. */
                        if (g_csd_ps_active) {
                            samplesOut[frameCount + i] = csd_l1;
                        } else {
                            samplesOut[frameCount + i] =
                                (unsigned)csd_ups_slot(1u, (int)samplesOut[frameCount + i]);
                        }
                        if (pmask_wire & 16u)
                        { unsigned seg3b; tseg :> seg3b; g_csd_seg[3] += seg3b - seg3a; }
                    }
#endif /* CSD_UPS_ASYNC */
                    /* REVERT(dp199-attr): DAC-R out slice (bit4). */
                    unsigned sb5a; if (pmask_wire & 16u) tseg :> sb5a;
                    if(XUA_I2S_N_BITS == 32)
                        p_i2s_dac[index++] <: bitrev(samplesOut[frameCount + i]);
                    else
                        partout(p_i2s_dac[index++], XUA_I2S_N_BITS, bitrev(samplesOut[frameCount + i]));
                    if (pmask_wire & 16u)
                    { unsigned sb; tseg :> sb; g_csd_seg_b[1] += sb - sb5a; }
                }
                } /* REVERT(dp200-emit-first): !csd_armed DAC-R block end */
#endif // (I2S_CHANS_DAC != 0)
                /* REVERT(dp199-attr): tail anchor: everything after the DAC-R
                 * out loop (dsd check, frame-boundary bookkeeping, exchange
                 * gate, tail floor latch) up to the loop bottom. */
                if (pmask_wire & 16u) tseg :> tailA;

            }  // !dsdMode


#if (DSD_CHANS_DAC != 0) && (NUM_USB_CHAN_OUT > 0)
            /* REVERT(dp200): armed-skip. The DoP/DSD detector votes on
             * samplesOut, which the armed mailbox pop owns (the host DoP
             * markers never reach it) -- armed DSD is not a supported
             * lane, and the check costs 12 t/frame on the accept-critical
             * iteration segment. Ratio-1 keeps it. */
            if(!csd_armed &&
               DoDsdDopCheck(dsdMode, dsdCount, curSamFreq, samplesOut, dsdMarker) == 0)
            {
#if (I2S_CHANS_ADC != 0) || (I2S_CHANS_DAC != 0)
                // Set clocks low
                p_lrclk <: 0;
                p_bclk <: 0;
#endif
                p_dsd_clk <: 0;
                return 0;
            }
#endif

#if (XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
            /* Increase frameCount by 2 since we have output two channels (per data line) */
            frameCount+=1;
            if(frameCount == I2S_CHANS_PER_FRAME)
#endif
            {
#if defined(CSD_UPS_ASYNC)
                /* Runtime ratio: one host pair every g_csd_usb_ratio LRCLK
                 * frames. The ADC side keeps writing every slot into
                 * samplesIn, so the USB mic sees the last pair of each cycle
                 * (1-in-8) -- the owner closed the ADC; the loopback tap is
                 * only armed at ratio 1. (Superseded for the mic side by the
                 * 2026-11-09 decimation phase control; see csd_mic_* below.) */
                /* 2026-11-09 (correction, same day): operand order fixed.
                 * My earlier "robustness" edit read
                 *     (g_csd_usb_ratio - 1u) >= audioToUsbRatioCounter
                 * which is TRUE on every frame, because the counter is reset
                 * to 0 after each transfer -- so DoSampleTransfer fired every
                 * LRCLK, the else branch never ran, and pacing collapsed to 1.
                 * That is the real explanation for the mic returning one frame
                 * per host frame with popsL == frames instead of 2*frames: it
                 * was never a pop-side fault. Measured by the subagent's host
                 * FSM harness: shipped order = 16 transfers / 16 frames at
                 * ratio 8; this order = 2 / 16.
                 * CHANGE: compare the counter AGAINST the ratio bound, so the
                 * counter climbs to (ratio - 1), matches once, transfers, and
                 * resets. '>=', not '==', keeps the stall property: if the
                 * ratio is lowered mid-cycle the counter is above the bound
                 * and transfers immediately rather than matching never.
                 * REVERT: restore the stock form
                 *     if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
                 * or, to keep the runtime ratio without the clamp property,
                 *     if (audioToUsbRatioCounter == (g_csd_usb_ratio - 1u))    */
                /* REVERT(dp169): the counter advances once per PASS
                 * (half-frame): one exchange per ratio WIRE frames = every
                 * 2*ratio passes. The dp168 gate counted passes as frames
                 * and exchanged at wire rate (277k/s instead of host-rate
                 * 96k pairs/s). */
                /* REVERT(dp171): counter ticks once per while-iteration = per WIRE frame;
                 * one exchange per ratio wire frames -> threshold ratio-1. */
                /* REVERT(dp199-attr): gate-entry slice (tailA..here = dsd
                 * check + TDM bookkeeping + the gate decision itself). */
                unsigned gateA = 0; if (pmask_wire & 16u) tseg :> gateA;
                if (pmask_wire & 16u)
                { unsigned sb; tseg :> sb; g_csd_seg_b[4] += sb - tailA; }
                if (audioToUsbRatioCounter >= (g_csd_usb_ratio - 1u))
#else
                if ((AUD_TO_USB_RATIO - 1) == audioToUsbRatioCounter)
#endif
                {
                    /* REVERT(dp199-attr): gate-false timing + gate-entry slice. */
                    /* REVERT(dp204): bit5-gated (was unconditional). The gate
                     * fires every iteration at ratio 1 (zero-margin wire
                     * loop): 2 shared-region RMWs ~10 t/iter. g_csd_w_exch
                     * stays unconditional as the lean-wire-rate oracle. */
                    if (pmask_wire & 32u) {
                        g_csd_gate_true++; g_csd_ratio_live = g_csd_usb_ratio;
                    }
                    unsigned seg5a; if (pmask_wire & 16u) tseg :> seg5a;                    /* Do samples transfer */
#ifdef CSD_UPS_ASYNC
                    /* PORT DECIMATOR per-cycle re-arm. The transfer boundary is
                     * where the pacing block resets audioToUsbRatioCounter, i.e.
                     * where a new N-word count cycle starts, so it is also where
                     * the decimator re-derives which count it presents. Runs once
                     * per N frames (6k/s at ratio 8 on this lane), never per word.
                     * REVERT: delete this #ifdef block. */
                    if (g_csd_usb_ratio > 1u) { xua_csd_decim_rearm(); }
#endif
#ifdef CSD_UPS_ASYNC
                    /* PREFETCH COLLECT (2026-11-10). Measured armed at ratio 8:
                     * host side healthy (ALSA hw_ptr 48000 frames/s) but the
                     * device consumed 1175 pairs/s with 97.7% of the window
                     * inside DoSampleTransfer and mean stall == worst stall ==
                     * 831 us -- a decouple round trip, not contention. Cause:
                     * at ratio 1 the exchange runs EVERY frame so a token is
                     * always in flight; moving it to every ratio-th frame left
                     * no token outstanding, so every exchange paid the full
                     * round trip and the software LRCLK collapsed to ~9 kHz.
                     * Keep consuming one pair per ratio frames (the host clock,
                     * per the owner's directive) but open the token on the
                     * PREVIOUS transfer frame so the ratio-1 idle wire frames
                     * hide the round trip. Channel protocol order is unchanged:
                     * one token, then its data. REVERT: delete this guard pair
                     * and the issue block below. */
                    if (g_csd_usb_ratio > 1u && g_csd_prefetch)
                    {
                        g_csd_prefetch = 0;
                        if (pmask_wire & 32u) { g_csd_prefetch_hits++; g_csd_xfer_tok++; } /* REVERT(dp168, dp191) */
                        unsigned pcmd = XUA_AUDCTL_NO_COMMAND;
#ifdef CSD_XFER_TIMING
                        CompleteSampleTransferTimed(c_aud, readBuffNo, pcmd, 1);
#else
                        CompleteSampleTransferUsbChans(c_aud, readBuffNo, pcmd);
#endif
                        if (pcmd != XUA_AUDCTL_NO_COMMAND)
                        {
                            return pcmd;
                        }
                    }
                    else
#endif
                    if (1)
                    {
                    /* The below looks a bit odd but forces the compiler to inline twice */
                    unsigned command;
                    if (pmask_wire & 32u) { g_csd_xfer_else++; } /* REVERT(dp168, dp191) */
#ifdef CSD_XFER_TIMING
                    /* HOW LONG DOES THE USB EXCHANGE STALL THE BIT-TIMED LOOP?
                     * The loop is proven clock-locked when the exchange is
                     * absent (disarmed probe: 44135 ADC reads/s against a
                     * 44.1 kHz LRCLK, i.e. exactly one iteration per frame) and
                     * proven throttled when it is present at ratio 8 (16655
                     * reads/s with reads/transfers exactly 8.0). This measures
                     * the stall instead of guessing: two timer reads and three
                     * stores per TRANSFER (once per ratio frames, ~48k/s worst
                     * case), never per slot. REVERT: delete this guard pair and
                     * the DIAG fields. */
                    timer xtmr; unsigned xt0, xt1;
                    xtmr :> xt0;
#endif
#ifdef CSD_NO_XFER
                    /* RATE PROBE (temporary, remove after use): perform no USB
                     * exchange at all -- same counter/flip behaviour, no
                     * channel traffic. If the loop then reaches the wire rate
                     * (~384k ADC reads/s), DoSampleTransfer is what throttles
                     * it to ~16.6k/s and the fix is to decouple the transfer
                     * from the bit-timed loop. If the rate is unchanged, the
                     * limiter is the port/clock path instead. Data is invalid
                     * in this build by design; only the counters matter. */
                    (void) underflowWord;
                    command = XUA_AUDCTL_NO_COMMAND;
#else
#ifdef CSD_XFER_TIMING
                    {
                        unsigned tcmd = XUA_AUDCTL_NO_COMMAND;
                        StartSampleTransfer(c_aud, underflowWord);
                        CompleteSampleTransferTimed(c_aud, readBuffNo, tcmd, 1);
                        command = tcmd;
                    }
#else
                    if(readBuffNo)
                        command = DoSampleTransfer(c_aud, 1, underflowWord);
                    else
                        command = DoSampleTransfer(c_aud, 0, underflowWord);
#endif
#endif

#ifdef CSD_XFER_TIMING
                    xtmr :> xt1;
                    {
                        unsigned xd = xt1 - xt0;
                        g_csd_xfer_n++;
                        g_csd_xfer_ticks += xd;
                        if (xd > g_csd_xfer_max) g_csd_xfer_max = xd;
                    }
#endif
                    if(command != XUA_AUDCTL_NO_COMMAND)
                    {
                        return command;
                    }
                    }

#ifdef CSD_UPS_ASYNC
                    /* Open the NEXT pair's token now, on this transfer frame, so
                     * decouple has the remaining ratio-1 wire frames to deliver
                     * it before the next collect. REVERT with the other
                     * CSD_UPS_ASYNC prefetch block. */
                    if (g_csd_usb_ratio > 1u && !g_csd_prefetch)
                    {
                        StartSampleTransfer(c_aud, underflowWord);
                        g_csd_prefetch = 1;
                    }
#endif
                /* Reset audio to usb counter because we have now completed one USB transfer and flip the ADC buffer */
                    audioToUsbRatioCounter = 0;
                    readBuffNo = !readBuffNo;
                    if (pmask_wire & 16u)
                    { unsigned seg5b; tseg :> seg5b; g_csd_seg[5] += seg5b - seg5a; }
                }
                else
                {
                    /* REVERT(dp199-attr): price the gate-false path. */
                    unsigned gfA = 0; if (pmask_wire & 16u) tseg :> gfA;
                    ++audioToUsbRatioCounter;
                    if (pmask_wire & 32u) { g_csd_gate_false++; } /* REVERT(dp191) */
                    if (pmask_wire & 16u)
                    { unsigned sb; tseg :> sb; g_csd_seg_b[5] += sb - gfA; csd_gf_n++; }
                }
                /* Reset the framecount because we have outputted all channels in the frame now */
                frameCount = 0;
                if (pmask_wire & 16u) g_csd_seg_n++;   /* REVERT(dp185) */
                /* REVERT(dp168 2026-11-13): wire-cadence floor, once per wire
                 * frame, unconditional, runtime-gated. WHY: armed (ratio>1) the
                 * loop had no per-frame wait — the exchange moved to every
                 * ratio-th frame — and free-ran at the mixer round-trip limit
                 * (measured 221-245k vs 384k wire), warping the mic stream.
                 * XS1 timer floor: an expired deadline costs nothing, so at
                 * ratio 1 this is a no-op load/store path, stock-preserving.
                 * wireSamFreq is the entry-time fabric rate (0 when the loop
                 * entered un-armed) -> fall back to the live wire rate. */
                if (g_csd_usb_ratio > 1u)
                {
                    if (wireTicks == 0u)
                    {
                        unsigned wf = csd_ups_diag_out_hz();
                        if (wf != 0u)
                        {
                            /* REVERT(dp169): the floor sits on the PASS
                             * path — one pass = one half-frame (one ADC
                             * read side + one LRCLK toggle) — so the unit
                             * is HALF a wire frame: stride = T/2 exactly
                             * (50M/wf ticks, remainder carried on wf). */
                            /* REVERT(dp171): one while-iteration = one FULL
                             * wire frame (measured: lrclk_emit = 2/pass at
                             * ratio-1, dp170 bench). Stride = full frame;
                             * the dp169 half-frame units starved the floor
                             * (130t < machine pass floor ~1700t -> f_skip
                             * 100% always). */
                            wireTicks = XS1_TIMER_HZ / wf;
                            wireRem   = XS1_TIMER_HZ % wf;
                            wireTmr :> wireNext;
                            g_csd_f_armed_n++;
                            g_csd_wire_ticks_val = wireTicks;
                            g_csd_wire_rem_val   = wireRem;
                        }
                    }

                }
                /* REVERT(dp184): per-frame tail timer block (tt_ticks/tt_n)
                 * removed — two timer reads + arithmetic on every wire
                 * frame for a diagnostic nobody reads (dp183 probe budget). */
                if (pmask_wire & 32u) { g_csd_c_tail++; } /* REVERT(dp168, dp191: bit5-gated) */
            }
            /* REVERT(dp199-attr): iteration total + tail slice. */
            if (pmask_wire & 16u)
            { unsigned lc; tseg :> lc;
              g_csd_seg_b[3] += lc - tailA; g_csd_loop_ticks += lc - loopA; }
            skip_ts_check = 0;
        }
    }
    return 0;
}

/* This helper function receives the command using the right transaction from Decouple when audiohub breaks */
static void receive_command(unsigned command,
                            chanend c_aud,
                            unsigned &curSamFreq,
                            unsigned &dsdMode,
                            unsigned &curSamRes_DAC,
                            unsigned &audioActive)
{
    debug_printf("receive_command: %d\n", command);
    /* REVERT(s15-diag): wake-code attribution (cold path). */
    g_csd_cmd_tot++; g_csd_cmd_last = command;
    if(command == XUA_AUDCTL_SET_SAMPLE_FREQ)
    {
        g_csd_cmd_sr++;
        curSamFreq = inuint(c_aud) * AUD_TO_USB_RATIO;
        debug_printf("receive_command set sr: %d\n", curSamFreq);
    }
    else if(command == XUA_AUD_SET_AUDIO_START)
    {
        g_csd_cmd_start++;
        /* Off = 0
         * DOP = 1
         * Native = 2
         */
        /* DSD is not a feature of this product (owner directive 2026-11-10,
         * "don't bother with DSD"; DSD_CHANS_DAC is commented out in
         * src/core/xua_defs.h:95). WHAT: consume the word decouple sends -- the
         * channel stream must stay aligned -- but force the mode off when the
         * build has no DSD channels. WHY: accepting decouple's value let
         * dsdMode become 2 (native) spontaneously after boot, which selected
         * numBits=32 and divide=32 and multiplied the rate by 32 in
         * AudioHwConfig's friendly-rate adjustment (in_hz=1536000 = 48000*32);
         * that run captured 0 nonzero mic words and refused to arm, while a
         * later stream in the SAME boot with dsdMode=0 was byte-exact. The
         * value originates from DoP/DoP-style payload detection, so random and
         * DC test stimuli can trigger it -- non-determinism by construction.
         * REVERT: dsdMode = inuint(c_aud);  (one line, channel order preserved
         * because the read still happens above). */
        /* Unconditional, NOT tied to DSD_CHANS_DAC. WHY: my first attempt
         * clamped only under #if (DSD_CHANS_DAC == 0); the very next run still
         * came up dsdMode=2 with numBits=32 and divide=32 (49152000/(48000*32))
         * on the first stream after boot, i.e. the guard did not hold the way
         * the headers suggested it would. Rather than chase the macro, the mode
         * is now dropped for this product outright, which is also exactly the
         * owner's instruction. Effect: dsdMode can never become non-zero, so
         * numBits stays XUA_I2S_N_BITS*I2S_CHANS_PER_FRAME and the x32
         * friendly-rate multiplication in AudioHwConfig can never fire.
         * REVERT: dsdMode = inuint(c_aud);  (the read above must stay so the
         * channel stream stays aligned). */
        unsigned csd_dsd_req = inuint(c_aud);
        dsdMode = DSD_MODE_OFF;
        (void) csd_dsd_req;
        curSamRes_DAC = inuint(c_aud);
        audioActive = 1;
        debug_printf("aud stream start\n");
    }
    else if (command == XUA_AUD_SET_AUDIO_STOP)
    {
        g_csd_cmd_stop++;   /* REVERT(s15-diag) */
        debug_printf("aud stream stop\n");
        if(XUA_LOW_POWER_NON_STREAMING)
        {
            audioActive = 0;
        }
    }
    else
    {
        g_csd_cmd_unh++;    /* REVERT(s15-diag) */
        debug_printf("aud unhandled cmd  %u\n", command);
    }
    /* Not we do not ACK back here - it is done when we re-start audio */
}

#if XUA_DFU_EN
#include "dfu_usb_server.h"
#endif


/* This function is a dummy version of the deliver thread that does not
   connect to the codec ports. It is used during DFU reset and during idle non-streaming mode, if enabled.
   Note there are two paths through depending on dfuMode.*/
[[combinable]]
static void dummy_deliver(chanend ?c_aud, unsigned sampFreq, unsigned dfuMode, unsigned &command)
{
    const int wait_ticks = XS1_TIMER_HZ / sampFreq;
    timer tmr;
    int tmr_trigger;
    tmr :> tmr_trigger;
    tmr_trigger += wait_ticks;

    if(!dfuMode)
    {
        StartSampleTransfer(c_aud, 0);
    }

/* This is a bit annoyting but we have to end in a while(1) select to be combinable */
#if ((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
#define COMPLETE_SAMPLE_TRANSFER(c, buf, cmd)   CompleteSampleTransferUsbChans(c, buf, cmd)
#else
#define COMPLETE_SAMPLE_TRANSFER(c, buf, cmd)   CheckForCmdNoUsbChans(c, buf, cmd)
#endif
    while (1)
    {
        /* Note, a select is used such that this task is combinable */
        select
        {
            case COMPLETE_SAMPLE_TRANSFER(c_aud, 0, command):   /* Check for command & transfer the samples & UBM */
                if(command)
                {
                    if(dfuMode)
                    {
                        /* Just consume the command, ignore it + keep on looping forever */
                        unsigned dummy1, dummy2, dummy3, dummy4;
                        receive_command(command, c_aud, dummy1, dummy2, dummy3, dummy4);
                        outct(c_aud, XS1_CT_END);
                    }
                    else
                    {
                        /* Process the command in the callee */
                        return;
                    }
                }
                /* Wait one sample period */
                tmr when timerafter(tmr_trigger) :> void;
                tmr_trigger += wait_ticks;

                /* Request more data/commands */
                if((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
                {
                    StartSampleTransfer(c_aud, 0);
                }
            break;
        }
    }
}


#if XUA_DFU_EN
/* External DFU handler task */
#include "dfu_usb_server.h"


/* Helper function to see if a request for entry to DFU has been issued via a sample rate change.
   If so, enter DFU. Note this code will never return. The device will
   need to be reset which is part of the DFU sequence */
void check_and_enter_dfu(unsigned curSamFreq, chanend c_aud, server interface i_dfu ?dfuInterface)
{
    /* Currently no more audio will happen after this point */
    if ((curSamFreq / AUD_TO_USB_RATIO) == AUDIO_STOP_FOR_DFU)
    {
        /* Handshake back the SR change command that put us in DFU*/
        outct(c_aud, XS1_CT_END);

        /* Request more data/commands */
        if((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
        {
            StartSampleTransfer(c_aud, 0);          /* Send first token to fire ISR in decouple */
        }

        unsigned command = XUA_AUDCTL_NO_COMMAND;
        while (1)
        {
           [[combine]]
            par
            {
#if (XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0)
                dfu_usb_server(dfuInterface);
#endif
                /* This never exits because we set DFU mode*/
                dummy_deliver(c_aud, 48000, 1, command);
            }
            /* Note, we shouldn't reach here. Audio, once stopped for DFU, cannot be resumed */
        }
    }
}
#endif /* XUA_DFU_EN */


void XUA_AudioHub(chanend ?c_aud, clock ?clk_audio_mclk, clock ?clk_audio_bclk,
    in port ?p_mclk_in,
    buffered _XUA_CLK_DIR port:32 ?p_lrclk,
    buffered _XUA_CLK_DIR port:32 ?p_bclk,
    buffered out port:32 (&?p_i2s_dac)[I2S_WIRES_DAC],
    buffered in port:32  (&?p_i2s_adc)[I2S_WIRES_ADC]
#if (XUA_SPDIF_TX_EN)
    , chanend c_spdif_out
#endif
#if (XUA_ADAT_RX_EN || XUA_SPDIF_RX_EN)
    , chanend c_dig_rx
#endif
#if ADJUSTABLE_MCLK_REQUIRED
    , chanend c_audio_rate_change
#endif
#if (XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0) && (XUA_DFU_EN == 1)
    , server interface i_dfu ?dfuInterface
#endif
#if (XUA_NUM_PDM_MICS > 0)
    , chanend c_pdm_in
#endif
)
{
/* This is a bit annoying but we have a mixture of nullable interfaces and variadic function signatures based on defines */
#if !((XUA_XUD_TILE_NUM != 0) && (XUA_AUDIO_IO_TILE_NUM == 0) && (XUA_DFU_EN == 1))
#define dfuInterface null
#endif
#if (XUA_ADAT_TX_EN)
    chan c_adat_out;
    unsigned adatSmuxMode = 0;
    unsigned adatMultiple = 0;
#endif
    unsigned curSamFreq = DEFAULT_FREQ * AUD_TO_USB_RATIO;
    unsigned curSamRes_DAC = STREAM_FORMAT_OUTPUT_1_RESOLUTION_BITS; /* Default to something reasonable */
    unsigned curSamRes_ADC = STREAM_FORMAT_INPUT_1_RESOLUTION_BITS; /* Default to something reasonable - note, currently this never changes*/
    unsigned command = XUA_AUDCTL_NO_COMMAND;
    unsigned mClk;
    unsigned divide;
    /* Flag used to indicate whether both interfaces are set to Alt 0 or not for power saving option
       This is only used when XUA_LOW_POWER_NON_STREAMING is defined to non-zero */
    unsigned audioActive = XUA_LOW_POWER_NON_STREAMING ? 0 : 1;
    /* This flag is to ensure that the decouple<->audio channel protocol is observed at startup.
       We need this because we hold off the ACK back to decouple as late as possible so that the control path knows audio is fully ready */
    unsigned firstRun = 1;

    while(1)
    {
        /* This is the "low power non-streaming" loop */
        while(audioActive == 0)
        {
            /* We only want to ACK after we have done our first transaction with decouple */
            if(firstRun == 0)
            {
                /* Handshake back previous command */
                if(XUA_USB_EN)
                {
                    outct(c_aud, XS1_CT_END);
                }
            }
            else
            {
                firstRun = 0;
            }
            /* Now run dummy loop with no IO. This is sufficient to poll for commands from decouple */
            dummy_deliver(c_aud, 1000, 0, command);  /* Run loop at 1kHz for min power and exit if command */
            receive_command(command, c_aud, curSamFreq, dsdMode, curSamRes_DAC, audioActive);
#if (XUA_DFU_EN == 1)
            check_and_enter_dfu(curSamFreq, c_aud, dfuInterface);
#endif /* (XUA_DFU_EN == 1) */
        } /* audioActive == 0 */

#if (DSD_CHANS_DAC > 0)
        /* Make sure the DSD ports are on and buffered - just in case they are not shared with I2S */
        EnableBufferedPort(p_dsd_clk, 32);
        for(int i = 0; i< DSD_CHANS_DAC; i++)
        {
            EnableBufferedPort(p_dsd_dac[i], 32);
        }
#endif

#if (MCLK_REQUIRED)
        xassert((!isnull(clk_audio_mclk) && !isnull(p_mclk_in)) && "Error: must provide non-null MCLK port and MCLK clock-block if digital Rx is enabled or XUA_AUDIO_IO_TILE_NUM==XUA_XUD_TILE_NUM");
        /* Clock master clock-block from master-clock port */
        configure_clock_src(clk_audio_mclk, p_mclk_in);
#if (XUA_ADAT_TX_EN)
        /* Set ADAT Tx port to be clock from master clock-block*/
        configure_out_port_no_ready(p_adat_tx, clk_audio_mclk, 0);
        set_clock_fall_delay(clk_audio_mclk, 7);
#endif
        /* If the XUD tile is different from AUDIO tile, then we start a clkblk for counting clocks on the XUD tile and start it in main.
        If XUD is on the same tile as AUDIO then we just connect p_for_mclk_count to the  clk_audio_mclk in main, but
        we need to start it here after all of the connections have been made.
        Note. we do not need a clk_audio_mclk if no dig Tx and XUD and AUDIO are on different tiles because I2S is driven by BCLK clkblk
        directly from the MCLK port. */
        /* Start the master clock-block */
        start_clock(clk_audio_mclk);
#endif /* (MCLK_REQUIRED) */

        /* Perform required CODEC/ADC/DAC initialisation */
        AudioHwInit();

        /* Only break this loop if LP non streaming enabled and streams are both Alt 0 */
        while((XUA_LOW_POWER_NON_STREAMING == 0) || audioActive)
        {
            /* Fabric rate (assistant, 2026-11-10). The I2S fabric -- BCLK, LRCLK
             * and therefore ADC1 acquisition -- must follow the WIRE rate, while
             * curSamFreq stays at the USB/host rate for endpoint sizing, feedback
             * and transfer pacing. Measured reason it must happen HERE: with the
             * fabric left on curSamFreq the armed run read DIVider?
             * 48000/49152000/16/64 (LRCLK 48 kHz) while the codec had been forced
             * to 384000, and the raw ADC1 witness advanced at only ~48.9k reads/s;
             * the mic could not contain wire data because there was no 384 kHz
             * clock on the port. Re-doing it after AudioHwConfig is fatal (see the
             * removed block). Decided in this branch on purpose: file-scope
             * publishes do not cross this loop's replicated par branches -- the
             * same trap xua_csd_set_usb_ratio() documents.
             * REVERT: replace fabricSamFreq with curSamFreq at the three use
             * sites below. */
            unsigned fabricSamFreq = curSamFreq;
#if defined(CSD_UPS_ASYNC)
            /* Condition is want, NOT enabled. WHY: the grant (enabled) is
             * issued inside AudioHwConfig(), which runs AFTER the ports are
             * configured -- requiring it here made the condition permanently
             * false and left DIVider? at 48000/16 in the armed state (measured).
             * The codec is forced to the family rate on `want` as well
             * (clearsounddac_board.xc csd_AudioHwConfig), so fabric and codec
             * still agree. The earlier fatal case was the second
             * ConfigAudioPortsWrapper call, not this condition.
             * REVERT: restore `&& csd_ups_is_enabled()`. */
            if (dsdMode == 0u && csd_ups_get_want())
            {
                unsigned famHz = csd_ups_expected_out_hz(curSamFreq);
                if (famHz != 0u)
                {
                    fabricSamFreq = famHz;
                }
            }
#endif
            g_csd_fabric_force_hz = fabricSamFreq;   /* DIAG mirror */

            /* Calculate what master clock we should be using */
            if (((MCLK_441) % fabricSamFreq) == 0)
            {
                mClk = MCLK_441;
#if (XUA_ADAT_TX_EN)
                /* Calculate ADAT SMUX mode (1, 2, 4) */
                adatSmuxMode = curSamFreq / 44100;
                adatMultiple = mClk / 44100;
#endif
            }
            else if (((MCLK_48) % fabricSamFreq) == 0)
            {
                mClk = MCLK_48;
#if (XUA_ADAT_TX_EN)
                /* Calculate ADAT SMUX mode (1, 2, 4) */
                adatSmuxMode = curSamFreq / 48000;
                adatMultiple = mClk / 48000;
#endif
            }

            /* Calculate master clock to bit clock (or DSD clock) divide for current sample freq
             * e.g. 11.289600 / (176400 * 64)  = 1 */
            {
                unsigned numBits = XUA_I2S_N_BITS * I2S_CHANS_PER_FRAME;

#if (DSD_CHANS_DAC > 0)
                if(dsdMode == DSD_MODE_DOP)
                {
                    /* DoP we receive in 16bit chunks */
                    numBits = 16;
                }
                else if(dsdMode == DSD_MODE_NATIVE)
                {
                    /* DSD native we receive in 32bit chunks */
                    numBits = 32;
                }
#endif
                divide = mClk / (fabricSamFreq * numBits);
                g_csd_div_curSamFreq = curSamFreq;   /* witness, see decl */
                g_csd_div_mClk = mClk;
                g_csd_div_divide = divide;
                g_csd_div_numBits = numBits;
                g_csd_div_dsdMode = (unsigned)dsdMode;

                // Do some checks
                xassert((divide > 0) && "Error: divider is 0, BCLK rate unachievable");

                unsigned remainder = mClk % ( fabricSamFreq * numBits);
                xassert((!remainder) && "Error: MCLK not divisible into BCLK by an integer number");

                /* Ignore special divide = 1 case */
                unsigned divider_is_odd = (divide & 0x1) && (divide != 1);
                xassert((!divider_is_odd) && "Error: divider is odd, clockblock cannot produce desired BCLK");

           }

#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
#if (DSD_CHANS_DAC > 0)
            if(dsdMode)
            {
                /* Configure audio ports */
                ConfigAudioPortsWrapper(
#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
                    p_dsd_dac,
                    DSD_CHANS_DAC,
#endif // (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                    p_i2s_adc,
                    I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                    null,
                    p_dsd_clk,
                    p_mclk_in, clk_audio_bclk, divide, curSamFreq);
            }
            else
#endif // (DSD_CHANS_DAC > 0)
            {
                ConfigAudioPortsWrapper(
#if (I2S_CHANS_DAC != 0)
                    p_i2s_dac,
                    I2S_WIRES_DAC,
#endif // (I2S_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                    p_i2s_adc,
                    I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                    p_lrclk,
                    p_bclk,
                    p_mclk_in, clk_audio_bclk, divide, fabricSamFreq);
            }
#endif // (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)

            {
                unsigned curFreq = curSamFreq;
#if (DSD_CHANS_DAC > 0)
                /* Make AudioHwConfig() implementation a little more user friendly in DSD mode...*/
                if(dsdMode == DSD_MODE_NATIVE)
                {
                    curFreq *= 32;
                }
                else if(dsdMode == DSD_MODE_DOP)
                {
                    curFreq *= 16;
                }
#endif
                /* Configure Clocking/CODEC/DAC/ADC for SampleFreq/MClk */

                /* User should mute audio hardware */
                AudioHwConfig_Mute();

                /* User code should configure audio harware for SampleFreq/MClk etc */

#if defined(__XS3A__) && (!ADJUSTABLE_MCLK_REQUIRED) && (XUA_USE_SW_PLL)
                sw_pll_fixed_clock(mClk); // output a fixed clock using the application PLL
#endif
                AudioHwConfig(curFreq, mClk, dsdMode, curSamRes_DAC, curSamRes_ADC);
#if (ADJUSTABLE_MCLK_REQUIRED)
                /* Notify clockgen of new mCLk */
                c_audio_rate_change <: mClk;
                c_audio_rate_change <: curFreq;

                /* Wait for ACK back from clockgen or ep_buffer to signal clocks all good */
                c_audio_rate_change :> int _;
#if XUA_USE_SW_PLL
                timer t;
                unsigned time;
                /* Allow some time for mclk to lock and MCLK to stabilise - this is important to avoid glitches at start of stream */
                t :> time;
                t when timerafter(time+40000000) :> void;
#endif

#endif
                /* User should unmute audio hardware */
                AudioHwConfig_UnMute();

/* 2026-11-10: the post-AudioHwConfig fabric re-clock was REMOVED.
 * WHY (measured): calling ConfigAudioPortsWrapper() a second time, after the
 * ports are already running, kills the audiohub task. Isolation proof, same
 * build with only that call removed: arecord/aplay return 0/0 with 768000
 * bytes captured (2 s at 48 kHz stereo) and DIVider? reads 48000/49152000/16;
 * with the call present the device refuses every set_params and every counter
 * stays at zero. The fabric rate must therefore be decided BEFORE the first
 * port configuration -- see fabricSamFreq above. REVERT: nothing to restore
 * here; the upstream decision is the implementation. */
            }

#if (XUA_NUM_PDM_MICS > 0)
            /* Send decimation factor to PDM task(s) */
            c_pdm_in <: curSamFreq / AUD_TO_MICS_RATIO;
#endif

            if(firstRun == 0)
            {
                /* TODO wait for good mclk instead of delay */
                /* No delay for DFU modes */
                if (command)
                {
#if 0
                    /* User should ensure MCLK is stable in AudioHwConfig */
                    if(retVal1 == XUA_AUDCTL_SET_SAMPLE_FREQ)
                    {
                        timer t;
                        unsigned time;
                        t :> time;
                        t when timerafter(time+AUDIO_PLL_LOCK_DELAY) :> void;
                    }
#endif
                    /* Handshake back from previous loop's command*/
                    if(XUA_USB_EN)
                    {
                        outct(c_aud, XS1_CT_END);
                    }
                }
            }
            firstRun = 0;

            par
            {

#if (XUA_ADAT_TX_EN)
                {
                    set_thread_fast_mode_on();
                    adat_tx_port(c_adat_out, p_adat_tx);
                }
#endif
                {
#if (XUA_SPDIF_TX_EN)
                    /* Communicate master clock and sample freq to S/PDIF thread */
                    outct(c_spdif_out, XS1_CT_END);
                    outuint(c_spdif_out, curSamFreq);
                    outuint(c_spdif_out, mClk);
                    if(curSamRes_DAC >= 24) {
                        outuint(c_spdif_out, 24);
                    }
                    else {
                        outuint(c_spdif_out, 16);
                    }
#endif

#if (XUA_ADAT_TX_EN)
                    // Configure ADAT parameters ...
                    //
                    // adat_oversampling =  256 for MCLK = 12M288 or 11M2896
                    //                   =  512 for MCLK = 24M576 or 22M5792
                    //                   = 1024 for MCLK = 49M152 or 45M1584
                    //
                    // adatSmuxMode   = 1 for FS =  44K1 or  48K0
                    //                = 2 for FS =  88K2 or  96K0
                    //                = 4 for FS = 176K4 or 192K0
                    outuint(c_adat_out, adatMultiple);
                    outuint(c_adat_out, adatSmuxMode);
#endif
                    command = AudioHub_MainLoop(c_aud
#if (XUA_SPDIF_TX_EN)
                       , c_spdif_out
#else
                       , null
#endif
#if (XUA_ADAT_TX_EN)
                       , c_adat_out
                       , adatSmuxMode
#endif
                       , divide, curSamFreq
#if (XUA_ADAT_RX_EN || XUA_SPDIF_RX_EN)
                       , c_dig_rx
#endif
#if (XUA_NUM_PDM_MICS > 0)
                       , c_pdm_in
#endif
                      , p_lrclk, p_bclk, p_i2s_dac, p_i2s_adc);

#if (XUA_USB_EN)
                    /* Now perform any additional inputs and update state accordingly */
                    receive_command(command, c_aud, curSamFreq, dsdMode, curSamRes_DAC, audioActive);
#if (XUA_DFU_EN == 1)
                    check_and_enter_dfu(curSamFreq, c_aud, dfuInterface);

#endif /* (XUA_DFU_EN == 1) */
#endif /* XUA_USB_EN */


#if XUA_NUM_PDM_MICS > 0
                    unsafe {
                        ma_shutdown((chanend_t)c_pdm_in); // shutdown mics
                    }
#endif

#if (XUA_ADAT_TX_EN)
#ifdef ADAT_TX_USE_SHARED_BUFF
                    /* Take out-standing handshake from ADAT core */
                    inuint(c_adat_out);
#endif
                    /* Notify ADAT Tx thread of impending new freq... */
                    outct(c_adat_out, XS1_CT_END);
#endif /* XUA_ADAT_TX_EN) */
                } /* AudioHub_MainLoop and command handler */
            } /* par */
        } /* while((!XUA_LOW_POWER_NON_STREAMING) || audioActive) */


        /* The following code can only be reached if XUA_LOW_POWER_NON_STREAMING is enabled and all streams stopped */

        /* First shutdown and reset ports before we may shutdown any clocks */
#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
#if (DSD_CHANS_DAC > 0)
        if(dsdMode)
        {
            /* Configure audio ports */
            DeConfigAudioPorts(
#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
                p_dsd_dac,
                DSD_CHANS_DAC,
#endif // (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                p_i2s_adc,
                I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                null,
                p_dsd_clk,
                p_mclk_in,
                clk_audio_bclk);
        }
        else
#endif // (DSD_CHANS_DAC > 0)
        {
            DeConfigAudioPorts(
#if (I2S_CHANS_DAC != 0)
                p_i2s_dac,
                I2S_WIRES_DAC,
#endif // (I2S_CHANS_DAC != 0)
#if (I2S_CHANS_ADC != 0)
                p_i2s_adc,
                I2S_WIRES_ADC,
#endif // (I2S_CHANS_ADC != 0)
                p_lrclk,
                p_bclk,
                p_mclk_in,
                clk_audio_bclk);
        }
#endif // (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)

#if (MCLK_REQUIRED)
        /* Start the master clock-block */
        stop_clock(clk_audio_mclk);
#endif

        /* Call user functions for core power down (eg. MCLK disable) and system component power down */
        AudioHwShutdown();

    } /* while(1)*/
}
