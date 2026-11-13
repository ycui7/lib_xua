// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.

#include "xua_commands.h"

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
        UserBufferManagement(samplesOut, samplesIn[readBuffNo]);
#endif

#if NUM_USB_CHAN_IN > 0
#pragma loop unroll
        for(int i = 0; i < NUM_USB_CHAN_IN; i++)
        {
            outuint(c_aud, samplesIn[readBuffNo][i]);
        }
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
