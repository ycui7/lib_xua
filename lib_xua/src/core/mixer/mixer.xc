// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#define XASSERT_UNIT MIXER
#include "xassert.h"

#include <xs1.h>
#include "xua.h"
#include "xua_commands.h"
#include "dbcalc.h"

/* FAST_MIXER has a bit of a nasty implentation but is more efficient */
#ifndef FAST_MIXER
#define FAST_MIXER   (1)
#endif

#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS) || !FAST_MIXER
#include "xc_ptr.h"
#endif

#if (XUA_MIXER_EN)

#if (XUA_OUT_VOLUME_IN_MIXER)
static unsigned int multOut_array[NUM_USB_CHAN_OUT + 1];
unsafe
{
    unsigned int volatile * unsafe multOut = multOut_array;
}
#endif

#if (XUA_IN_VOLUME_IN_MIXER)
static unsigned int multIn_array[NUM_USB_CHAN_IN + 1];
unsafe
{
    unsigned int volatile * unsafe multIn = multIn_array;
}
#endif

#if defined (LEVEL_METER_LEDS) || defined (LEVEL_METER_HOST)
static unsigned abs(int x)
{
#if 0
    if (x < 0)
        return x*-1;
    return x;
#else
    int const mask = x >> sizeof(int) * 8 - 1;
    return (x + mask) ^ mask;
#endif
}
#endif

static const int SOURCE_COUNT = NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + XUA_MAX_MIX_COUNT + 1;

static int samples_array[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + XUA_MAX_MIX_COUNT + 1]; /* One larger for an "off" channel for mixer sources" */
#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
/**
 * vol_ctld_samples_to_device_array[NUM_USB_CHAN_OUT]
 *
 * Only used when XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX
 * Contains post-mix, post-volume-control samples that are stored linearly for giving to audio
 * in GiveSamplesToDevice()
 */
static int vol_ctld_samples_to_device_array[NUM_USB_CHAN_OUT];
#endif
static int samples_to_host_map_array[NUM_USB_CHAN_IN];
static int samples_to_device_map_array[NUM_USB_CHAN_OUT];

unsafe
{
    int volatile * const unsafe ptr_samples = samples_array;
    int volatile * const unsafe samples_to_host_map = samples_to_host_map_array;
    int volatile * const unsafe samples_to_device_map = samples_to_device_map_array;
#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
    int volatile * const unsafe ptr_vol_ctld_samples_to_device = vol_ctld_samples_to_device_array;
#endif
}

#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER)
/* ClearSound function generator, PRE-mixer injection. DEFINED BY THE
 * APPLICATION (sw_usb_audio/clearsound_uac/src/extensions/csd_gen.xc) and
 * declared here rather than #included, so the library keeps no compile-time
 * dependency on application sources -- the same arrangement main.xc uses for
 * the CSD_DIAG_MIDI entry point. Parameter types must mirror the definition
 * exactly, or the compiler rejects the call as unknown.
 *
 * Scalars in, scalar out, both directions: this file is library code and the
 * mixer's tile placement is lib_xua's choice, so a pointer here would be
 * correct only by accident of placement. The condition is spelled literally
 * (matching CSD_GEN_PREMIX in csd_gen.h) rather than via that macro because
 * this TU cannot include the app header -- if the two conditions ever drift,
 * BOTH threads could advance the generator's NCO and every tone lands at
 * exactly twice its commanded frequency, which is why the app rig tests the
 * mixer build's counters explicitly. */
void csd_gen_mix_begin(int hostL, int hostR, unsigned dsd_mode);
int  csd_gen_mix_sample(unsigned ch, int host);
void csd_mb_note_vol(unsigned index, unsigned val);
void csd_mb_note_mix_frame(void);   /* proof-of-life, csd_gen_mailbox.c */
void csd_mb_note_mix_busy(unsigned delta_ticks);  /* audio-wait window peak  */
void csd_mb_note_dac_sample(unsigned ch, unsigned node, int sample);
void csd_mb_note_host_nonzero(unsigned ch, int sample, unsigned frame);
/* Node ids DUPLICATED from csd_gen.h on purpose (same pattern as the
 * CSD_GEN_PREMIX condition above): this TU cannot include the application
 * header. If they drift, the instrument reports node labels wrong -- audio
 * is untouched, so the cost of drift is a confusing rig, not a fault. Keep
 * in sync. */
#define CSD_NODE_HOST    0u   /* received from host (GetSamplesFromHost)      */
#define CSD_NODE_POSTMIX 1u   /* read from ptr_samples post-mix/pre-volume    */
#define CSD_NODE_VOLUME  2u   /* after the output-volume MAC                  */
#define CSD_NODE_DAC     3u   /* the word mixer1 handed to the audio thread   */
#define CSD_NODE_TOKEN   4u   /* trigger token from the audio thread          */
/* Diagnostic frame counter for the host-node first-nonzero timing. File
 * scope because GetSamplesFromHost() is its own inline function, not mixer1
 * text; written and read only from the mixer thread. */
static unsigned csd_mix_frame_ix = 0;
#endif

#if (XUA_MAX_MIX_COUNT > 0)
int mix_mult_array[XUA_MAX_MIX_COUNT * XUA_MIX_INPUTS];
#if (FAST_MIXER == 0)
int mix_map_array[XUA_MAX_MIX_COUNT * XUA_MIX_INPUTS];
#endif

unsafe
{
    int volatile * const unsafe mix_mult = mix_mult_array;
#if (FAST_MIXER == 0)
    int volatile * const unsafe mix_map = mix_map_array;
#endif
}

#define slice(a, i) (a + i * XUA_MIX_INPUTS)

#endif

#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
/* Arrays for level data */
int samples_to_host_inputs[NUM_USB_CHAN_IN];            /* Audio transmitted to host i.e. device inputs */
xc_ptr samples_to_host_inputs_ptr;

#ifdef LEVEL_METER_LEDS
int samples_to_host_inputs_buff[NUM_USB_CHAN_IN];       /* Audio transmitted to host i.e. dev inputs */
xc_ptr samples_to_host_inputs_buff_ptr;
#endif
static int samples_from_host_streams[NUM_USB_CHAN_OUT]; /* Peak samples for audio stream from host */

static int samples_mixer_outputs[XUA_MAX_MIX_COUNT];        /* Peak samples out of the mixer */
xc_ptr samples_mixer_outputs_ptr;

static inline void ComputeMixerLevel(int sample, int i)
{
    int x;
    int y;
    xc_ptr ptr;

    x = abs(sample);

    /* y = samples_mixer_outputs[i] */
    read_via_xc_ptr_indexed(y, samples_mixer_outputs_ptr, i);

    if(x > y)
    {
        /* samples_to_host_outputs[i] = x; */
        write_via_xc_ptr_indexed(samples_mixer_outputs_ptr,i,x);
    }
}
#endif


#if (FAST_MIXER)
void setPtr(int src, int dst, int mix);
int doMix0(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix1(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix2(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix3(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix4(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix5(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix6(volatile int * const unsafe samples, volatile int * const unsafe mult);
int doMix7(volatile int * const unsafe samples, volatile int * const unsafe mult);
#else
#pragma unsafe arrays
static inline int doMix(volatile int * unsafe samples, volatile int * unsafe const mixMap, volatile int * const unsafe mult)
{
    int h=0;
    int l=0;

#pragma loop unroll
    for (int i=0; i<XUA_MIX_INPUTS; i++)
    unsafe{
        int sample;
        int source;
        int weight;
        read_via_xc_ptr_indexed(source, mixMap, i);
        sample = samples[source];
        read_via_xc_ptr_indexed(weight, mult, i);


        {h,l} = macs(sample, weight, h, l);
    }

    /* Perform saturation */
    l = sext(h, XUA_MIXER_MULT_FRAC_BITS);

    if(l != h)
    {
        if(h>>32)
            h = (0x80000000>>7);
        else
            h = (0x7fffff00>>7);
    }
    return h<<7;
}
#endif

#pragma unsafe arrays
static inline void GiveSamplesToHost(chanend c, volatile int * unsafe hostMap)
{
#if (XUA_IN_VOLUME_IN_MIXER && XUA_IN_VOLUME_AFTER_MIX)
    int mult;
    int h;
    unsigned l;
#endif

#pragma loop unroll
    for (int i=0; i<NUM_USB_CHAN_IN; i++)
    {
        int sample;

#if (XUA_MAX_MIX_COUNT > 0)
        unsafe
        {
            sample = ptr_samples[hostMap[i]];
        }
#else
        unsafe
        {
            sample = ptr_samples[i + NUM_USB_CHAN_OUT];
        }
#endif

#if (XUA_IN_VOLUME_IN_MIXER && XUA_IN_VOLUME_AFTER_MIX)
#warning IN Vols in mixer, AFTER mix & map

        unsafe
        {
            mult = multIn[i];
        }
        {h, l} = macs(mult, sample, 0, 0);

        //h <<= 3 done on other side */

        outuint(c, h);
#else
        outuint(c,sample);
#endif
    }
}

#pragma unsafe arrays
static inline void GetSamplesFromHost(chanend c)
{
#if (NUM_USB_CHAN_OUT == 0)
    inuint(c);
#else
    {
#pragma loop unroll
        for (int i=0; i<NUM_USB_CHAN_OUT; i++)
        unsafe {
            int sample, x;
#if (XUA_OUT_VOLUME_IN_MIXER && !XUA_OUT_VOLUME_AFTER_MIX)
            int mult;
            int h;
            unsigned l;
#endif
            /* Receive sample from decouple */
            sample = inuint(c);

#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
            /* Compute peak level data */
            x = abs(sample);
            if(x > samples_from_host_streams[i])
            {
                samples_from_host_streams[i] = x;
            }
#endif

#if (XUA_OUT_VOLUME_IN_MIXER && !XUA_OUT_VOLUME_AFTER_MIX)
#warning OUT Vols in mixer, BEFORE mix & map
            mult = multOut[i];
            {h, l} = macs(mult, sample, 0, 0);
            h<<=3;
#if (STREAM_FORMAT_OUTPUT_RESOLUTION_32BIT_USED == 1)
            h |= (l >>29)& 0x7; // Note: This step is not required if we assume sample depth is 24bit (rather than 32bit)
                                // Note: We need all 32bits for Native DSD
#endif
            sample = h;
#endif
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
            csd_mb_note_dac_sample((unsigned)i, CSD_NODE_HOST, sample);
            csd_mb_note_host_nonzero((unsigned)i, sample, csd_mix_frame_ix);
#endif
            ptr_samples[i] = sample;
        }
    }
#endif
}

#pragma unsafe arrays
static inline void GiveSamplesToDevice(chanend c, volatile int * unsafe deviceMap)
{
#if (NUM_USB_CHAN_OUT == 0)
    outuint(c, 0);
#else
#pragma loop unroll
    for (int i=0; i<NUM_USB_CHAN_OUT; i++)
    {
        int sample;
#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
        unsafe {
            sample = ptr_vol_ctld_samples_to_device[i];
        }
#else
#if (XUA_MAX_MIX_COUNT > 0)
        /* If mixer turned on sort out the channel mapping */
        unsafe
        {
            /* Read index to sample from the map then Read the actual sample value */
            sample = ptr_samples[deviceMap[i]];
        }
#else
        unsafe
        {
            /* Read the actual sample value */
            sample = ptr_samples[i];
        }
#endif
#endif
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
        /* DELIBERATELY AFTER every #if above: a hook inside one volume path
         * would read 0 -- a false "words are clean" -- whenever the build
         * takes the other path. Four nodes total on this path (see the
         * CSD_NODE_* defines); removed once case A is attributed. */
        csd_mb_note_dac_sample((unsigned)i, CSD_NODE_DAC, sample);
#endif
        outuint(c, sample);
    }
#endif
}

#pragma unsafe arrays
static inline void GetSamplesFromDevice(chanend c)
{
#if (XUA_IN_VOLUME_IN_MIXER && !XUA_IN_VOLUME_AFTER_MIX)
    int mult;
    int h;
    unsigned l;
#endif

#pragma loop unroll
    for (int i=0; i<NUM_USB_CHAN_IN; i++)
    {
        int sample;
        int x;
        int old_x;
        sample = inuint(c);

#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
        /* Compute peak level data */
        x = abs(sample);

        // old_x = samples_to_host_inputs[i]
        read_via_xc_ptr_indexed(old_x, samples_to_host_inputs_ptr, i);
        if(x > old_x)
        {
            //samples_to_host_inputs[i] = x;
            write_via_xc_ptr_indexed(samples_to_host_inputs_ptr, i, x);
        }
#endif

#if (XUA_IN_VOLUME_IN_MIXER && !XUA_IN_VOLUME_AFTER_MIX)
        /* Volume processing - read relevant multiplier */
        unsafe
        {
            mult = multIn[i];
        }

        /* Do the multiply */
        {h, l} = macs(mult, sample, 0, 0);
        h <<= 3;
        sample = h;
#endif
        unsafe
        {
            assert((XUA_MIXER_OFFSET_IN + i) < (NUM_USB_CHAN_IN + NUM_USB_CHAN_OUT));
            ptr_samples[XUA_MIXER_OFFSET_IN + i] = sample;
        }
    }
}

#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
/**
 * do_output_volume_control()
 *
 * If XUA_OUT_VOLUME_IN_MIXER and XUA_OUT_VOLUME_AFTER_MIX enabled, apply volume control
 * on a USB OUT sample after mixing and before sending it to audio.
 * The volume control output is written to the ptr_vol_ctld_samples_to_device[] array
 * from where it is read linearly in GiveSamplesToDevice().
 *
 * - Input: out_ch_index = index of the USB output channel
 * - Output: The scaled sample is written linearly into
 *   ptr_vol_ctld_samples_to_device[out_ch_index]
 */
#pragma unsafe arrays
static inline void do_output_volume_control(int out_ch_index)
{
    int sample;
    int mult;
    int h;
    unsigned l;

    unsafe
    {
#if (XUA_MAX_MIX_COUNT > 0)
        unsafe
        {
            /* If mixer turned on, account for channel mapping.
            Read index to sample from the map then Read the actual sample value */
            sample = ptr_samples[samples_to_device_map[out_ch_index]];
        }
#else
        unsafe
        {
            /* Read the actual sample value */
            sample = ptr_samples[out_ch_index];
        }
#endif
        mult = multOut[out_ch_index];
    }
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
    csd_mb_note_dac_sample((unsigned)out_ch_index, CSD_NODE_POSTMIX, sample);
#endif

    /* Do volume control processing */
    {h, l} = macs(mult, sample, 0, 0);
    h<<=3;              // Shift used to be done in audio thread but now done here incase of 32bit support
#if (STREAM_FORMAT_OUTPUT_RESOLUTION_32BIT_USED == 1)
    h |= (l >>29)& 0x7; // Note: This step is not required if we assume sample depth is 24bit (rather than 32bit)
                        // Note: We need all 32bits for Native DSD
#endif
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
    csd_mb_note_dac_sample((unsigned)out_ch_index, CSD_NODE_VOLUME, h);
#endif
    unsafe
    {
        ptr_vol_ctld_samples_to_device[out_ch_index] = h;
    }
}
#endif


#pragma unsafe arrays
static void mixer1(chanend c_host, chanend c_mix_ctl, chanend ?c_mixer2, chanend c_audio)
{
    int cmd_pending_flag = 0;
    int pending_cmd[3]; // command + a max of 2 extra values

    int mixer1_mix2_flag = (DEFAULT_FREQ > 96000);
#if (XUA_MAX_MIX_COUNT > 0)
    int mixed;
#endif
#if (XUA_MAX_MIX_COUNT > 0) || (XUA_IN_VOLUME_IN_MIXER) || (XUA_OUT_VOLUME_IN_MIXER) || defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
    unsigned cmd;
    unsigned char ct;
#endif
    unsigned request = 0;
#if (XUA_MAX_MIX_COUNT > 0)
    /**
     * Tracks whether mixer2 was triggered during the last
     * sample exchange. If the decoupler responds with a
     * control command, mixer2 is not triggered.
     */
    int mixer2_triggered = 1;
#endif
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER)
    /* Stream DSD mode as the host declared it (0 off, 1 DoP, 2 native, from
     * XUA_AUD_SET_AUDIO_START). The generator's pre-mixer gate shuts on any
     * non-zero value: native DSD needs all 32 payload bits untouched -- the
     * volume paths above carry the same warning for the same reason. DoP is
     * ALSO caught by the generator's own raw-frame vote (host alt settings
     * declare DoP as PCM, so this variable alone would miss it); both
     * mechanisms run, erring toward "is DSD". */
    unsigned csd_dsd_mode = 0;
    /* Hoisted out of the loop body: a timer declared inside would be
     * re-acquired every sample period (an alloc bt per frame on the very
     * thread whose frame budget this instrument measures). */
    timer csd_t_mix;
    unsigned csd_t_mix_t0, csd_t_mix_t1;
#endif

    while (1)
    {
#if (XUA_MAX_MIX_COUNT > 0)
        if(mixer2_triggered)
        {
            inuint(c_mixer2); // synchronise with mixer2
            mixer2_triggered = 0;
        }
#endif
        /* Request from audio()/mixer2() */
        request = inuint(c_audio);
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
        csd_t_mix :> csd_t_mix_t0;
        /* Tile[1] resource question, measured: audio() BLOCKS between sending
         * this token and getting the sample exchange back, and its I2S port
         * deadline is the hard one (xua_audiohub.xc:192 "If mixer1 misses the
         * sample period, this port timestamp..."). The window that matters is
         * therefore inuint(c_audio) -> GiveSamplesToDevice: everything mixer1
         * does with the audio thread waiting (generator NCO, volume MACs, the
         * instrument itself). Peak of that delta in timer ticks, 12500 ticks
         * = one 48 kHz frame at 600 MHz fast-mode.
         *
         * The timer is hoisted to the loop scope on purpose: a resource
         * declared in the loop body is re-acquired every frame (an alloc bt
         * per sample period on a thread whose budget we are measuring). */
#endif
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER)
        /* ---- case-A fix: the PCM prefill word must be SILENCE ----
         *
         * This token is echoed by the decouple as the sample for every
         * channel while it pre-buffers (decouple.xc handle_audio_request:
         * outuint(c_mix_out, underflowSample)). The audiohub derives it from
         * its dsdMode: 0x96969696 is lib_xua's native-DSD underflow word
         * (xua_audiohub.xc:228), and a DSD-silent word is NOT silent for
         * PCM -- it is a full-scale-ish pattern (peak measured +1.4 dBFS
         * into the DAC path) held for the whole prefill, ~13 266 words =
         * 276 ms, once per boot. That is the acoustic gate's case A: mic
         * hears +11.7/+12.9 dB over the no-stream floor on a stream of
         * digital zeros, confined to the FIRST stream after load, while
         * every digital node downstream faithfully carries it (instrument:
         * rig/dac_node_probe2.py, MIXer:DACPeak? at 5 nodes).
         *
         * The mixer knows this stream's DSD mode (XUA_AUD_SET_AUDIO_START,
         * csd_dsd_mode below), so it is the right place to enforce the
         * invariant: prefill in PCM => the substituted sample is zero.
         * DSD modes keep the word untouched -- for them 0x96.. IS silence
         * and the DoP/native gates rely on it.
         */
        if (csd_dsd_mode == 0u && request != 0u)
        {
            request = 0u;
        }
#endif
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
        /* The trigger token doubles as the decouple's OUT-prefill word:
         * handle_audio_request echoes the word it received as the sample for
         * every channel while outUnderflow. So whatever value the audio
         * thread sent HERE is what the host node sees during prefill. The
         * audiohub sends `underflowWord`, which is 0x96969696 when its
         * local dsdMode read NATIVE at audio() entry (xua_audiohub.xc:228)
         * -- the exact word caught at the host node on the first stream.
         * Peaking the token separates 'audiohub sent the DSD word' (fix the
         * word / the mode latch) from 'decouple substituted something'. */
        csd_mb_note_dac_sample(0u, CSD_NODE_TOKEN, (int)request);
        csd_mix_frame_ix++;
        /* Proof-of-life for THIS loop, once per sample period. See
         * csd_mb_note_mix_frame (csd_gen_mailbox.c): the case-A ladder
         * implicated mixer activity; if a streamed zero file leaves this at
         * zero, mixer1 never ran and nothing downstream of here can be the
         * source. */
        csd_mb_note_mix_frame();
#endif

        // Check if cmd is pending from last exchange with decoupler.
        // If no command pending, transfer sample with audiohub to unblock it as
        // soon as possible. This is esp important for the I2S TDM case where lots
        // more samples need to be exchanged in lesser time than regular I2S
        // (for example, for 96KHz tdm8. 4 i2s lines, exchange 32 samples in (1/(96000*8))seconds)
        if(!cmd_pending_flag)
        {
            GiveSamplesToDevice(c_audio, samples_to_device_map);
            GetSamplesFromDevice(c_audio);
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
            csd_t_mix :> csd_t_mix_t1;
            csd_mb_note_mix_busy(csd_t_mix_t1 - csd_t_mix_t0);
#endif
        }
        else
        {
            // forward cmd from last exchange with decoupler to audio
            switch(pending_cmd[0])
            {
                case XUA_AUDCTL_SET_SAMPLE_FREQ:
                    outct(c_audio, pending_cmd[0]); // cmd
                    outuint(c_audio, pending_cmd[1]); // samp freq
                    break;

                case XUA_AUD_SET_AUDIO_START:
                    /* Inform mixer2 (or audio()) about change */
                    outct(c_audio, pending_cmd[0]); // cmd
                    outuint(c_audio, pending_cmd[1]); // dsd mode
                    outuint(c_audio, pending_cmd[2]); // sample res
                    break;

                case XUA_AUD_SET_AUDIO_STOP:
                    /* Pass on command */
                    outct(c_audio, pending_cmd[0]); // cmd
                    break;

                default:
                    break;
            }
            chkct(c_audio, XS1_CT_END);
            outct(c_host, XS1_CT_END); // Only now ack back to the decoupler.
            cmd_pending_flag = 0;
            continue;
        }

        /* Forward on Request for data to decouple thread, only after sample exchange with audiohub is complete */
        outuint(c_host, request);

        /* Between request to decouple and response ~ 400nS latency for interrupt to fire */
#if (XUA_MAX_MIX_COUNT > 0) || (XUA_IN_VOLUME_IN_MIXER) || (XUA_OUT_VOLUME_IN_MIXER) || defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
        select
        {
            /* Check if EP0 intends to send us a control command */
            case inct_byref(c_mix_ctl, ct):
            {
                int mix, index, val;

                /* Handshake back to tell EP0 we are ready for an update */
                outct(c_mix_ctl, XS1_CT_END);

                /* Receive command from EP0 */
                cmd = inuint(c_mix_ctl);

                /* Interpret control command */
                switch (cmd)
                {
#if (XUA_MAX_MIX_COUNT > 0)
                    case SET_SAMPLES_TO_HOST_MAP:
                        {
                            int dst = inuint(c_mix_ctl);
                            int src = inuint(c_mix_ctl);
                            inct(c_mix_ctl);

                            assert((dst < NUM_USB_CHAN_IN) && msg("Host map destination out of range"));
                            assert((src < SOURCE_COUNT) && msg("Host map source out of range"));

                            if((dst < NUM_USB_CHAN_IN) && (src < SOURCE_COUNT))
                            {
                                unsafe
                                {
                                    samples_to_host_map[dst] = src;
                                }
                            }
                        }
                        break;

                    case SET_SAMPLES_TO_DEVICE_MAP:
                        {
                            int dst = inuint(c_mix_ctl);
                            int src = inuint(c_mix_ctl);
                            inct(c_mix_ctl);

                            assert((dst < NUM_USB_CHAN_OUT) && msg("Device map destination out of range"));
                            assert((src < SOURCE_COUNT) && msg("Device map source out of range"));

                            if((dst < NUM_USB_CHAN_OUT) && (src < SOURCE_COUNT))
                            {
                                unsafe
                                {
                                    samples_to_device_map[dst] = src;
                                }
                            }
                        }
                        break;

                    case SET_MIX_MULT:
                        mix = inuint(c_mix_ctl);
                        index = inuint(c_mix_ctl);
                        val = inuint(c_mix_ctl);
                        inct(c_mix_ctl);

                        assert((mix < XUA_MAX_MIX_COUNT) && msg("Mix mult mix out of range"));
                        assert((index < XUA_MIX_INPUTS) && msg("Mix mult index out of range"));

                        if((index < XUA_MIX_INPUTS) && (mix < XUA_MAX_MIX_COUNT))
                        {
                            unsafe
                            {
                                mix_mult[(mix * XUA_MIX_INPUTS) + index] = val;
                            }
                        }
                        break;

                    case SET_MIX_MAP:
                        {
                            unsigned mix = inuint(c_mix_ctl);
                            unsigned input = inuint(c_mix_ctl);     /* mixer input */
                            unsigned source = inuint(c_mix_ctl);    /* source */
                            inct(c_mix_ctl);

                            assert((mix < XUA_MAX_MIX_COUNT) && msg("Mix map mix out of range"));
                            assert((input < XUA_MIX_INPUTS) && msg("Mix map index out of range"));
                            assert((source < SOURCE_COUNT) && msg("Mix map source out of range"));

                            if((input < XUA_MIX_INPUTS) && (mix < XUA_MAX_MIX_COUNT) && (source < SOURCE_COUNT))
                            {
#if (FAST_MIXER)
                                setPtr(input, source, mix);
#else
                                unsafe
                                {
                                    mix_map[(mix * XUA_MIX_INPUTS) + input] = source;
                                }
#endif
                            }
                        }
                        break;
#endif /* if XUA_MAX_MIX_COUNT > 0 */

#if (XUA_IN_VOLUME_IN_MIXER)
                    case SET_MIX_IN_VOL:
                        index = inuint(c_mix_ctl);
                        val = inuint(c_mix_ctl);
                        inct(c_mix_ctl);

                        assert((index  < (NUM_USB_CHAN_IN + 1)) && msg("In volume index out of range"));

                        if(index < NUM_USB_CHAN_IN + 1)
                        {
                            unsafe
                            {
                                multIn[index] = val;
                            }
                        }
                        break;
#endif
#if (XUA_OUT_VOLUME_IN_MIXER)
                    case SET_MIX_OUT_VOL:
                        index = inuint(c_mix_ctl);
                        val = inuint(c_mix_ctl);
                        inct(c_mix_ctl);
#if defined(CSD_GEN_EN) && defined(CSD_GEN_INSTRUMENT) && defined(MIXER) && (MIXER)
                        /* Diagnostic, app-provided (csd_gen_mailbox.c): counts
                         * the host volume commands that actually arrive HERE,
                         * which is the question no earlier instrument could
                         * answer -- amixer caches, raw control transfers never
                         * landed, and this build prints nothing to the host.
                         * Compiled out of every non-CSD build, so lib_xua's
                         * stock behaviour is unchanged. */
                        csd_mb_note_vol((unsigned)index, val);
#endif

                        assert((index  < (NUM_USB_CHAN_OUT + 1)) && msg("Out volume index out of range"));

                        if(index < NUM_USB_CHAN_OUT + 1)
                        {
                            unsafe
                            {
                                multOut[index] = val;
                            }
                        }
                        break;
#endif

#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
                    /* Peak samples of stream from host to device (via USB) */
                    case GET_STREAM_LEVELS:
                        index = inuint(c_mix_ctl);
                        chkct(c_mix_ctl, XS1_CT_END);
                        outuint(c_mix_ctl, samples_from_host_streams[index]);
                        outct(c_mix_ctl, XS1_CT_END);
                        samples_from_host_streams[index] = 0;
                        break;
#endif
                }
                break;
            }
            default:
                /* Select default */
                break;
        } // select
#endif

        /* Get response from decouple */
        if(testct(c_host))
        {
            int sampFreq;
            unsigned command = inct(c_host);
            cmd_pending_flag = 1; // set pending flag. Command will be forwarded to audihub in the next sample period.
            pending_cmd[0] = command;

            switch(command)
            {
                case XUA_AUDCTL_SET_SAMPLE_FREQ:
                    sampFreq = inuint(c_host);
                    mixer1_mix2_flag = sampFreq > 96000;
                    pending_cmd[1] = sampFreq;
                    break;

                case XUA_AUD_SET_AUDIO_START:
                    /* Inform mixer2 (or audio()) about change */
                    unsigned dsd_mode = inuint(c_host);
                    unsigned sample_res = inuint(c_host);
                    pending_cmd[1] = dsd_mode;
                    pending_cmd[2] = sample_res;
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER)
                    csd_dsd_mode = dsd_mode;
#endif
                    break;

                case XUA_AUD_SET_AUDIO_STOP:
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER)
                    /* Fail-closed: a stale non-zero would permanently mute
                     * the test tone; a stale zero can only let the tone
                     * through while the generator's own raw-frame DoP vote
                     * still guards the payload. Belt AND braces, in that
                     * order. */
                    csd_dsd_mode = 0;
#endif
                    break;

                default:
                    break;
            }

#pragma loop unroll
            /* Reset the mix values back to 0 */
            for (int i=0; i<XUA_MAX_MIX_COUNT; i++)
            {
                unsafe
                {
                    ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + i] = 0;
                }
            }
        }
        else
        {
            GetSamplesFromHost(c_host);
#if defined(CSD_GEN_EN) && defined(MIXER) && (MIXER)
            /* Function generator, PRE-mixer: SUM the wave into the host
             * stream's mixer sources before doMix() reads them, so the tone
             * rides the same mix matrix, routing and post-mix volume as real
             * host audio (user requirement: "before the mixer, summed to the
             * host audio stream"). Sum, never overwrite -- the mixer must
             * still see the host stream to mix it. Saturation, DoP voting and
             * the DSD gate all live in the application; this line is a data
             * hand-off and nothing else.
             *
             * ptr_samples is written here inside mixer1's own window (after
             * the mixer2 sync at the loop top, before this iteration's
             * outuint(c_mixer2)), the same window GetSamplesFromHost itself
             * writes in -- so the lock-step handshake that protects doMix
             * protects this too. */
            unsafe
            {
                int csd_h0 = (NUM_USB_CHAN_OUT > 0) ? ptr_samples[0] : 0;
                int csd_h1 = (NUM_USB_CHAN_OUT > 1) ? ptr_samples[1] : 0;
                csd_gen_mix_begin(csd_h0, csd_h1, csd_dsd_mode);
#pragma loop unroll
                for (int i = 0; i < NUM_USB_CHAN_OUT; i++)
                {
                    /* Read through a temp rather than `a[i] = f(i, a[i])`:
                     * this codebase has already measured XC rejecting the
                     * single-expression form through the unsafe accessors
                     * (store(x, load(x)+1)), and sequencing costs nothing
                     * here. */
                    int csd_in = ptr_samples[i];
                    int csd_out = csd_gen_mix_sample((unsigned)i, csd_in);
                    ptr_samples[i] = csd_out;
                }
            }
#endif
            GiveSamplesToHost(c_host, samples_to_host_map);

#if (XUA_MAX_MIX_COUNT > 0)
            /* Trigger mixer2 */
            outuint(c_mixer2, mixer1_mix2_flag);
            mixer2_triggered = 1;

            /* Do the mixing */
            unsafe
            {
#if (FAST_MIXER)
                mixed = doMix0(ptr_samples, slice(mix_mult, 0));
#else
                mixed = doMix(ptr_samples, slice(mix_map, 0), slice(mix_mult, 0));
#endif
                ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + 0] = mixed;
            }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
            ComputeMixerLevel(mixed, 0);
#endif
#if (MAX_FREQ > 96000)
            if (!mixer1_mix2_flag)
#endif
            {

#if (XUA_MAX_MIX_COUNT > 2)
                unsafe
                {
#if (FAST_MIXER)
                    mixed = doMix2(ptr_samples, slice(mix_mult, 2));
#else
                    mixed = doMix(ptr_samples, slice(mix_map, 2), slice(mix_mult, 2));
#endif
                    ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + 2] = mixed;
                }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
                ComputeMixerLevel(mixed, 2);
#endif
#endif

#if (XUA_MAX_MIX_COUNT > 4)
                unsafe
                {
#if (FAST_MIXER)
                    mixed = doMix4(ptr_samples, slice(mix_mult, 4));
#else
                    mixed = doMix(ptr_samples, slice(mix_map, 4), slice(mix_mult, 4));
#endif
                    ptr_samples[XUA_MIXER_OFFSET_MIX + 4] = mixed;
                }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
                ComputeMixerLevel(mixed, 4);
#endif
#endif

#if (XUA_MAX_MIX_COUNT > 6)
                unsafe
                {
#if (FAST_MIXER)
                    mixed = doMix6(ptr_samples, slice(mix_mult, 6));
#else
                    mixed = doMix(ptr_samples, slice(mix_map, 6), slice(mix_mult, 6));
#endif
                    ptr_samples[XUA_MIXER_OFFSET_MIX + 6] = mixed;
                }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
                ComputeMixerLevel(mixed, 6);
#endif
#endif
            }
#endif
#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
        #warning OUT Vols in mixer, AFTER mix & map
        #pragma loop unroll
        for (int i=0; i<NUM_USB_CHAN_OUT; i++)
        {
            int ch_index;
            unsafe {
        #if (XUA_MAX_MIX_COUNT > 0)
            // when mixer2 is present, mixer1 only does volume control for the channels that are not present with mixer2.
            // these are the odd numbered mixer output channels.
            // If ch_index is one of those channels, do volume control in mixer1.
            ch_index = samples_to_device_map[i];
            int do_vol_control = 0;
            if(ch_index >= (NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN)) /* if its one of the mixer output channels */
            {
                if(!((ch_index - (NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN)) & 0x1)) /* if its on even numbered mixer channel */
                {
                    do_vol_control = 1;
                }
            }
        #else // there's no second mixer. mixer1 does all the volume control
            int do_vol_control = 1;
        #endif
            if(do_vol_control)
            {
                do_output_volume_control(i);
            }
            }
        }
    #endif
        }
    }
}

#if (XUA_MAX_MIX_COUNT > 0)
#pragma unsafe arrays
static void mixer2(chanend c_mixer1)
{
    int mixed;
    outuint(c_mixer1, 0); // To get mixer1 started
    int mixer2_mix2_flag;
    while (1)
    {
        mixer2_mix2_flag = inuint(c_mixer1);

        /* Do the mixing */
#if (XUA_MAX_MIX_COUNT > 1)
        unsafe
        {
#if (FAST_MIXER)
            mixed = doMix1(ptr_samples, slice(mix_mult, 1));
#else
            mixed = doMix(ptr_samples, slice(mix_map, 1), slice(mix_mult, 1));
#endif
            ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + 1] = mixed;
        }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
        ComputeMixerLevel(mixed, 1);
#endif
#endif

#if (MAX_FREQ > 96000)
        /* Fewer mixes when running higher than 96kHz */
        if (!mixer2_mix2_flag)
#endif
        {
#if (XUA_MAX_MIX_COUNT > 3)
            unsafe
            {
#if (FAST_MIXER)
                mixed = doMix3(ptr_samples, slice(mix_mult, 3));
#else
                mixed = doMix(ptr_samples, slice(mix_map, 3), slice(mix_mult, 3));
#endif
                ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + 3] = mixed;
            }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
            ComputeMixerLevel(mixed, 3);
#endif
#endif

#if (XUA_MAX_MIX_COUNT > 5)
            unsafe
            {
#if (FAST_MIXER)
                mixed = doMix5(ptr_samples, slice(mix_mult, 5));
#else
                mixed = doMix(ptr_samples, slice(mix_map, 5), slice(mix_mult, 5));
#endif
                ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + 5] = mixed;

            }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
            ComputeMixerLevel(mixed, 5);
#endif
#endif

#if (XUA_MAX_MIX_COUNT > 7)
            unsafe
            {
#if (FAST_MIXER)
                mixed = doMix7(ptr_samples, slice(mix_mult, 7));
#else
                mixed = doMix(ptr_samples, slice(mix_map, 7), slice(mix_mult, 7));
#endif
                ptr_samples[NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + 7] = mixed;
            }
#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
            ComputeMixerLevel(mixed, 7);
#endif
#endif
        }

    // mixer2 is responsible for output volume control, unless the sample is not available with mixer2,
    // in which case it leaves it to mixer1 to do.
    // mixer2 has ptr_samples[0..(NUM_USB_CHAN_OUT+NUM_USB_CHAN_IN)] as well as index 1,3,5 and 7 (odd indexes!) of the mixed outputs

#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
        #warning OUT Vols in mixer, AFTER mix & map
        #pragma loop unroll
        for (int i=0; i<NUM_USB_CHAN_OUT; i++)
        {
            int ch_index;
            unsafe {
            ch_index = samples_to_device_map[i];
            }
            int do_vol_control = 0;
            if(ch_index < (NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN)) /* if ch_index is one of the usb out or in channels*/
            {
                do_vol_control = 1;
            }
            else
            {
                if((ch_index - (NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN)) & 0x1) /* if ch_index is an odd numbered mixer output channel*/
                {
                    do_vol_control = 1;
                }
            }
            if(do_vol_control)
            {
                do_output_volume_control(i);
            }
        }
#endif

        outuint(c_mixer1, 0);
    }
}
#endif

void mixer(chanend c_mix_in, chanend c_mix_out, chanend c_mix_ctl)
{
#if (XUA_MAX_MIX_COUNT > 0)
    chan c;
#endif

#if defined (LEVEL_METER_HOST) || defined(LEVEL_METER_LEDS)
    samples_to_host_inputs_ptr = array_to_xc_ptr((samples_to_host_inputs, unsigned[]));
#ifdef LEVEL_METER_LEDS
    samples_to_host_inputs_buff_ptr = array_to_xc_ptr((samples_to_host_inputs, unsigned[]));
#endif
    samples_mixer_outputs_ptr = array_to_xc_ptr((samples_mixer_outputs, unsigned[]));
#endif

    /* Zero EVERY sample the mixer can put on the wire, not only ptr_samples.
     *
     * Bug (measured on ClearSoundDac, mixer build, MIXER=1 MAX_MIX_COUNT=4):
     * GiveSamplesToDevice() reads ptr_vol_ctld_samples_to_device[i] whenever
     * XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX -- both default 1
     * with MIXER on (xua_conf_default.h:1467/1477). That array was written
     * only by do_output_volume_control(), which runs only once a host frame
     * arrives, and was never initialised here. From boot to the first frame
     * the mixer therefore sent uninitialized static RAM down the I2S/DAC
     * path. The board's own capture tap caught it verbatim: a frozen
     * 0x69696970 (-1.7 dBFS) at every range from 10 ms to 2 s on a cold
     * board with NO stream ever opened, clean on the second open (the array
     * had been written by then). That is the first-open pop the acoustic
     * gate measures as case A, and why it survived every codec-register
     * theory: it is digital content, upstream of the chip.
     *
     * Second hole in the same line: the loop bound was
     * OUT + IN + MAX_MIX_COUNT while samples_array has one MORE element --
     * the "off" channel at SOURCE_COUNT-1 that the mute/default mix maps
     * point at. doMix() can read it as a source, so it had to be zeroed too.
     */
    for (int i=0;i<NUM_USB_CHAN_OUT + NUM_USB_CHAN_IN + XUA_MAX_MIX_COUNT + 1;i++)
    unsafe {
        ptr_samples[i] = 0;
    }
#if (XUA_OUT_VOLUME_IN_MIXER && XUA_OUT_VOLUME_AFTER_MIX)
    for (int i=0; i<NUM_USB_CHAN_OUT; i++)
    unsafe {
        ptr_vol_ctld_samples_to_device[i] = 0;
    }
#endif

    for (int i=0; i<NUM_USB_CHAN_OUT; i++)
    {
        samples_to_device_map_array[i] = i;
    }

#if (XUA_OUT_VOLUME_IN_MIXER)
    for (int i=0; i<NUM_USB_CHAN_OUT; i++)
    unsafe{
        multOut[i] = MAX_VOLUME_MULT;
    }
#endif

#if (XUA_IN_VOLUME_IN_MIXER)
    for (int i=0; i<NUM_USB_CHAN_IN; i++)
    unsafe{
        multIn[i] = MAX_VOLUME_MULT;
    }
#endif

    for (int i=0; i<NUM_USB_CHAN_IN; i++)
    unsafe{
        samples_to_host_map[i] = XUA_MIXER_OFFSET_IN + i;
    }

#if (XUA_MAX_MIX_COUNT> 0)
    for (int i=0;i<XUA_MAX_MIX_COUNT;i++)
        for (int j=0;j<XUA_MIX_INPUTS;j++)
        unsafe{
#if (FAST_MIXER == 0)
            mix_map[i * XUA_MIX_INPUTS + j] = (j < 16 ? j : j + 2);
#endif
            mix_mult[i * XUA_MIX_INPUTS + j] = (i==j ? db_to_mult(0, XUA_MIXER_DB_FRAC_BITS, XUA_MIXER_MULT_FRAC_BITS) : 0);
        }
#endif

    par
    {
#if (XUA_MAX_MIX_COUNT > 0)
        mixer1(c_mix_in, c_mix_ctl, c, c_mix_out);
        mixer2(c);
#else
        mixer1(c_mix_in, c_mix_ctl, null, c_mix_out);
#endif
    }
}

#endif
