// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include "xua_commands.h"

#ifdef CSD_UPS_ASYNC
/* REVERT(dp200-wire-diet): returns 1 when the exchange DEFERS UBM to the
 * producer core (armed ratio>1 + ECHO 2 -- the shipping armed loopback
 * mode, where UBM writes no mic slots; the port decimator publishes the
 * ADC words directly). Publishes a copy of the host pair on the seqlock
 * consumed by csd_ub_consume_if_new() (csd_ps_ring.c). The wire's own
 * samplesOut is re-used by the mailbox pop, hence the copy. All ops
 * ldw/stw: the producer is a separate tile core (main.xc dp167 note),
 * cross-thread, and the compiler must not hoist any of it. */
extern unsigned g_csd_usb_ratio;
extern unsigned g_csd_ups_echo_mode;
extern unsigned g_csd_hb;   /* dp224 host-frame flag (set at c_aud complete, csd_ps_ring.c def) */
extern unsigned g_csd_ub_seq;
extern int g_csd_ub_pair[2];
extern unsigned g_csd_probe_mask; /* REVERT(bench r8) */
extern unsafe chanend g_cePop;   /* dp225e: single pop chan global */
extern unsigned g_csd_wire_iter;   /* dp225b: main-loop heartbeat (vendor TU) */   /* dp225: opened by the audiohub task; 'unsafe' storage like the proven uc_audiohw1 (plain file-scope chanend is rejected) */
static inline unsigned csd_ub_defer_handoff(const unsigned samplesOut[])
{
    /* 2026-11-22 (clean-tone campaign): defer whenever ARMED (ratio>1), not
     * only under ECHO 2. Measured on the bench: armed ECHO 0 is SILENT at the
     * DAC for every preset (-100 dBFS at the mic, all rates) — the inline
     * wire-side UBM (~8 us partner wake) doubles the per-frame cost against a
     * 260 t budget and throttles the wire to ~16.6k/s (xua_audiohub.xc
     * CSD_XFER_TIMING note), starving the FD feed. ECHO 2 already deferred
     * and sounded. ECHO 1 (feed-IN mirror) keeps the stock wire-side UBM
     * call — that mode's whole point is mirroring the wire-side feed words.
     * Mic semantics at armed ECHO 0 are unaffected for the host: the pop-side
     * publish (csd_dsp.xc armed block) carries the last wire pair into the
     * mic slots, and the deferred UBM's mic-slot writes land in the
     * producer's scratch s_ub_mic exactly as in the ECHO 2 mode.
     * 2026-11-22 (r2 clean-tone leg): ratio-2 was tried INLINE (the stock
     * pre-defer path): SILENT (-100 dBFS, wire throttled to 16.6k/s — the
     * ~8 us partner wake per exchange still starves the bit-timed loop at
     * r2's exchange rate). DEFERRED r2 sounds but fails the clean-tone gate
     * (comb f0-n*189.4 Hz, -30 dBFS, deterministic; seed 3/4 invariant, task
     * feed reps ~0) — an unresolved r2 rate-servo/pacing wall, documented,
     * NOT a reason to re-starve the wire. r4/r8 deferred measure CLEAN
     * (41.1/41.5 dB margins). */
    if (g_csd_usb_ratio <= 1u || g_csd_ups_echo_mode == 1u) { return 0; }
    if (g_csd_probe_mask & 128u) { return 0; } /* REVERT(bench r8): bit7 forces wire-side UBM under ECHO 2 */
    /* dp220 2026-11-23 (owner directive: diagnostics must not touch the
     * timed path): publish ONE slot per HOST frame, not per exchange. The
     * exchange runs at fabric rate (384k) and the XUD side replays the
     * held pair on the R-1 filler exchanges; publishing the replay made
     * the consumer see R copies of every pair. Counter-free cadence: a
     * new pair is only available once every g_csd_usb_ratio exchanges. */
    /* dp223f/g (measured + arithmetic): ONE main-loop pass emits BOTH L and
     * R port frames, so the exchange runs at wire/2 = 192k for every
     * family ratio, while the true host-frame rate is wire/R. Publish
     * cadence = every (R/2)-th exchange  =>  host-rate, exact:
     * r2: every 1 (192k host), r4: every 2, r8: every 4. (dp220A's /R
     * over-divided r2 to 96k = half feed -> wire honest-repeat 28 %;
     * publish-every-exchange would feed r8's held XUD replays 4x.)
     * Even ratios only (2/4/8) — the family set. */
    /* dp224: cadence = the TRUE host-frame event, not a guessed divide.
     * g_csd_hb is set once per completed USB OUT transfer (the c_aud
     * CompleteSampleTransferUsbChans site in xua_audiohub.xc, same
     * sanctioned functional region) and consumed here. The main-loop pass
     * rate free-runs with the port buffers (measured 169k-300k for a
     * 384k wire), so every /R formula drifts against the host grid and
     * amplitude-modulates the feed (r2 comb: 172 Hz sideband family,
     * 656/828/999 Hz, tone -41 vs combs -38). 3 plain memory ops. */
    if (g_csd_hb == 0u) { return 1; }   /* replay exchange: swallow */
    g_csd_hb = 0u;
    /* dp225f: VALUE-DEDUP REMOVED. It keyed on samplesOut equality; a
     * constant test stream (/dev/zero, silence) has every GENUINE pair
     * equal, so the heuristic swallowed 8191 of every 8192 true host
     * frames (publish collapsed to 30/s, feed starved, engine repeated
     * boot input). The queue's own seqlock makes replays harmless: the
     * engine consumes one pair per two output frames at the same exchange
     * cadence, so phase advance is structurally exact without dedup. */
    /* dp220b: on the TRUE host frame, publish below AND return 0 so the
     * vendor's inline UserBufferManagement(samplesOut,..) runs this one
     * exchange — its armed fast path (csd_dsp.xc dp220) feeds the FD FIR.
     * Vendor-designed call site, host-rate cadence, ~15 t body. */
    /* dp223c: plain C seqlock on volatile canonical objects (see the
     * claim block note in xua_audiohub.xc for why dp[] asm cannot work
     * across tile/task data segments). */
    unsigned a = samplesOut[0], b = samplesOut[1];
    unsigned sq = g_csd_ub_seq + 1u;
    g_csd_ub_seq = sq;                 /* odd: window */
    g_csd_ub_pair[0] = (int)a;
    g_csd_ub_pair[1] = (int)b;
    g_csd_ub_seq = sq + 1u;            /* even: done */
    return 0;   /* dp220b: vendor inline UBM runs on the true host frame */}
#endif

static inline void StartSampleTransfer(chanend ?c_aud, const unsigned underflowWord)
{
    if(XUA_USB_EN)
    {
        outuint(c_aud, underflowWord);
    }
}


#pragma unsafe arrays
#pragma select handler
static inline void CompleteSampleTransferUsbChans(chanend ?c_aud, const int readBuffNo, unsigned &command)
{
    /* Check for sample freq change (or other command) or new samples from mixer*/
    if(testct(c_aud))
    {
        command = inct(c_aud);
#ifndef CODEC_MASTER
        if(dsdMode == DSD_MODE_OFF)
        {
#if (I2S_CHANS_ADC != 0 || I2S_CHANS_DAC != 0)
            /* Set clocks low */
            p_lrclk <: 0;
            p_bclk <: 0;
#endif
        }
        else
        {
#if(DSD_CHANS_DAC != 0)
            /* DSD Clock might not be shared with lrclk or bclk... */
            p_dsd_clk <: 0;
#endif
        }
#endif
#if (DSD_CHANS_DAC > 0)
        if(dsdMode == DSD_MODE_DOP)
            dsdMode = DSD_MODE_OFF;
#endif
            return;
    }
    else
    {
#if NUM_USB_CHAN_OUT > 0
#pragma loop unroll
        for(int i = 0; i < NUM_USB_CHAN_OUT; i++)
        {
            int tmp = inuint(c_aud);
            samplesOut[i] = tmp;
        }
#else
        inuint(c_aud);
#endif
        const unsigned pmask_x = g_csd_probe_mask; /* REVERT(dp202): one load
            * per exchange (see wire-loop hoist note in xua_audiohub.xc) */
        /* Run user code */
#ifdef CSD_XFER_TIMING
        { /* 2026-11-12: inline UBM cost probe (xt_out/xt_out_n, otherwise
           * dead INFO fields, 10 ns ticks). Separates 'UBM work' from
           * 'partner wake' in the 7.9 us transfer frame. REVERT with
           * CSD_XFER_TIMING. */
          timer tubm; unsigned b0, b1;
          tubm :> b0;
          UserBufferManagement(samplesOut, samplesIn[readBuffNo]);
          tubm :> b1;
          unsigned du = b1 - b0;
          g_csd_xt_out += du; g_csd_xt_out_n++;
        }
#else
#ifdef CSD_UPS_ASYNC
        /* REVERT(dp200-wire-diet): armed+ECHO2 defers UBM to the producer. */
        if (!csd_ub_defer_handoff(samplesOut))
#endif
        UserBufferManagement(samplesOut, samplesIn[readBuffNo]);
#endif

#if NUM_USB_CHAN_IN > 0
#pragma loop unroll
        for(int i = 0; i < NUM_USB_CHAN_IN; i++)
        {
            /* REVERT(dp181): ship-site zero-birth witness (see decl in
             * xua_audiohub.xc; this header is included by that TU only).
             * REVERT(dp186): counter stores gated behind probe mask bit5. */
            unsigned csd_sw = samplesIn[readBuffNo][i];
            if ((pmask_x & 32u) && (unsigned)i < 2u) {  /* REVERT(dp202): mask
                * snapshot hoisted above the exchange -- see wire-loop note */
                g_csd_w_shipn[i]++;
                if (csd_sw == 0u) g_csd_w_shipz[i]++;
            }
            outuint(c_aud, csd_sw);
        }
        g_csd_w_exch++;  /* one stw/exchange: kept unconditional -- it is the
                          * lean-wire-rate oracle that caught the dp202 tax */
#endif
    }

    command = XUA_AUDCTL_NO_COMMAND;
}


#pragma select handler
static inline void CheckForCmdNoUsbChans(chanend ?c_aud, const int readBuffNo, unsigned &command)
{
    /* Poll for CT coming in from USB */ 
    /* In this case USB is still enabled, even though we have no audio channels to/from
     * host. Check for cmd - only expecting STOP_AUDIO_FOR_DFU. */
    unsigned char cmd;
    select
    {
        case inct_byref(c_aud, cmd):
            command = (unsigned)cmd;
            break;
        default:
            command = XUA_AUDCTL_NO_COMMAND;
            break; 
    }

    /* Run user code */
    UserBufferManagement(samplesOut, samplesIn[readBuffNo]);
}



#ifdef CSD_UPS_ASYNC
/* Two-phase exchange, receive half (2026-11-12). WHY: in master mode
 * (CODEC_MASTER==0) HandleSampleClock is compiled out — the loop's ONLY
 * synchronisation against the decouple node is the token exchange, and the
 * IN half (2 inuint) waits on the decouple RESPONSE, whose wake latency
 * (~1.85 us measured) lands mid-frame and drags the whole loop to 4.5 us
 * when the exchange runs every frame. Splitting the exchange across two
 * consecutive transfer frames costs nothing at ratio>1 (both frames carry
 * the same pair) and removes one response wait from each frame. The two
 * inuint are adjacent on the wire, so receiving them in the frame AFTER
 * the outuints is protocol-identical: decouple sends its reply as one
 * burst and the channel buffers it. Order discipline: StartPair OUT half,
 * then the reply of the PREVIOUS pair (ReceivePrevReply), then StartPair
 * the next. REVERT: delete this function and the CSD_UPS_ASYNC call sites. */
static inline void ReceivePrevReply(chanend ?c_aud, const int readBuffNo)
{
    if(XUA_USB_EN)
    {
        for(int i = 0; i < NUM_USB_CHAN_OUT; i++)
        {
            int tmp = inuint(c_aud);
            samplesOut[i] = tmp;
        }
        /* 2026-11-12: UBM moved OUT of this exchange (see the CSD_UPS_ASYNC
         * block at the transfer-frame tail in xua_audiohub.xc). WHY: the
         * timed witnesses prove the channel ops cost ~44 ns total (in 19 ns,
         * out 25 ns, link hidden by the prefetch) while the transfer frame
         * costs 7.9 us -- the difference is the per-exchange CROSS-FILE
         * UBM call (hot-path rule: a cross-file call per wire-rate frame
         * blows the budget; measured again here). The channel protocol only
         * requires the mic words to be sent after the reply pair; those
         * words are staged by the preceding UBM call (one frame stale by
         * design now, like the OUT path already is). Order at the transfer
         * frame becomes: reply in, mic out (staged), UBM refill for next
         * frame, start next token. REVERT: restore this call, delete the
         * block at the call site. */
#ifdef CSD_UPS_ASYNC
        /* no UBM here at ratio>1 -- see transfer-frame tail */
#else
        UserBufferManagement(samplesOut, samplesIn[readBuffNo]);
#endif
    }
}
#endif

#ifdef CSD_XFER_TIMING
/* Timed clone of CompleteSampleTransferUsbChans (see that function for the
 * uninstrumented original -- keep the two in step). The armed-ratio-8 loop
 * spends 97.6% of its time in here and a prefetch of the token did not change
 * the rate (ADC reads stayed 9397/s), so the wait is inside one of the three
 * channel operations below and must be identified, not assumed:
 *   ct   = the testct/inuint command probe
 *   in   = the NUM_USB_CHAN_OUT inuint loop (host -> device audio)
 *   out  = the NUM_USB_CHAN_IN outuint loop (device -> host audio)
 * Only one of these can be the ~831 us. REVERT: delete this function, its
 * guards, and the six DIAG fields it feeds. */
static inline void CompleteSampleTransferTimed(chanend ?c_aud, const int readBuffNo,
                                               unsigned &command, unsigned do_in)
{
    timer t0; unsigned a, b;

    t0 :> a;
    if(testct(c_aud))
    {
        command = inct(c_aud);
#ifndef CODEC_MASTER
        if(dsdMode == DSD_MODE_OFF)
        {
#if (I2S_CHANS_ADC != 0 || I2S_CHANS_DAC != 0)
            p_lrclk <: 0;
            p_bclk <: 0;
#endif
        }
#endif
        t0 :> b; g_csd_xt_ct += (b - a); g_csd_xt_ct_n++;
        return;
    }
    t0 :> b; g_csd_xt_ct += (b - a); g_csd_xt_ct_n++;

    t0 :> a;
#if NUM_USB_CHAN_OUT > 0
#pragma loop unroll
    for(int i = 0; i < NUM_USB_CHAN_OUT; i++)
    {
        int tmp = inuint(c_aud);
        samplesOut[i] = tmp;
    }
#else
    inuint(c_aud);
#endif
    t0 :> b; g_csd_xt_in += (b - a); g_csd_xt_in_n++;

    /* The port-timestamp monitor says one LRCLK emit per transfer stretches to
     * 20419 port ticks (831 us) while every other emit hits its 32-tick budget,
     * yet the three channel operations above/below total microseconds. The only
     * code in this exchange that is not a channel operation is the user
     * callback, which on this lane runs the upsampler fill (csd_ups_host_pair)
     * and the mic decimator. It was outside every timed region -- timed now.
     * REVERT with the rest of CSD_XFER_TIMING. */
    t0 :> a;
#ifdef CSD_UPS_ASYNC
    /* REVERT(dp200-wire-diet): armed+ECHO2 defers UBM to the producer
     * (xt_ubm then reads ~0 on the armed leg -- expected). */
    if (!csd_ub_defer_handoff(samplesOut))
#endif
    UserBufferManagement(samplesOut, samplesIn[readBuffNo]);
    t0 :> b; g_csd_xt_ubm += (b - a); g_csd_xt_ubm_n++;
    if ((b - a) > g_csd_xt_ubm_max) g_csd_xt_ubm_max = (b - a);

    t0 :> a;
#if NUM_USB_CHAN_IN > 0
#pragma loop unroll
    for(int i = 0; i < NUM_USB_CHAN_IN; i++)
    {
        outuint(c_aud, samplesIn[readBuffNo][i]);
    }
#endif
    t0 :> b; g_csd_xt_out += (b - a); g_csd_xt_out_n++;

    command = XUA_AUDCTL_NO_COMMAND;
}
#endif /* CSD_XFER_TIMING */

static inline unsigned DoSampleTransfer(chanend ?c_aud, const int readBuffNo, const unsigned underflowWord)
{
    if(XUA_USB_EN)
    {
        unsigned command;
        if((NUM_USB_CHAN_OUT > 0) || (NUM_USB_CHAN_IN > 0))
        {
            StartSampleTransfer(c_aud, underflowWord);                  /* Send first token to fire ISR in decouple */
            CompleteSampleTransferUsbChans(c_aud, readBuffNo, command); /* Check for command & transfer the samples & UBM */
        }
        else
        {
            CheckForCmdNoUsbChans(c_aud, readBuffNo, command); /* Check for command & UBM */
        }

        return command;
    }
    else
    {
        /* I2S without USB - just run user code */
        UserBufferManagement(samplesOut, samplesIn[readBuffNo]);

        return XUA_AUDCTL_NO_COMMAND;
    }
}
