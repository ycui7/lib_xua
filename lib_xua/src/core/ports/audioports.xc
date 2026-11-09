// Copyright 2011-2026 XMOS LIMITED.
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
#include <xs1.h>
#include <platform.h>
#include "xua.h"
#include "audioports.h"
#include "xassert.h"

void ConfigAudioPorts(
#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
    buffered out port:32 p_i2s_dac[],
    int numPortsDac,
#endif

#if (I2S_CHANS_ADC != 0)
    buffered in port:32  p_i2s_adc[],
    int numPortsAdc,
#endif

#if (CODEC_MASTER == 0)
    buffered out port:32 ?p_lrclk,
    buffered out port:32 p_bclk,
#else
    in port ?p_lrclk,
    in port p_bclk,
#endif
    in port ?p_mclk_in,
    clock clk_audio_bclk, unsigned int divide, unsigned curSamFreq)
{
#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)

#if (CODEC_MASTER == 0)

    /* We always expect a mclk pin when xcore is master */
    assert(!isnull(p_mclk_in));

#ifdef __XS3A__
  	/* Increase drive strength of clock ports to 8mA */
	asm volatile ("setc res[%0], %1" :: "r" (p_bclk), "r" (0x200006));
    if(!isnull(p_lrclk))
    {
	    asm volatile ("setc res[%0], %1" :: "r" (p_lrclk), "r" (0x200006));
    }
#endif

    /* Note this call to stop_clock() will pause forever if the port clocking the clock-block is not low.
     * deliver() should return with this being the case */
    stop_clock(clk_audio_bclk);

    /* When in DSD mode p_lrclk is null, so check here (allows code reuse) */
    if(!isnull(p_lrclk))
    {
        clearbuf(p_lrclk);
    }
    clearbuf(p_bclk);

#if (I2S_CHANS_ADC != 0)
    for(int i = 0; i < numPortsAdc; i++)
    {
        clearbuf(p_i2s_adc[i]);
    }
#endif

#if (I2S_CHANS_DAC != 0)|| (DSD_CHANS_DAC != 0)
    for(int i = 0; i < numPortsDac; i++)
    {
        clearbuf(p_i2s_dac[i]);
    }
#endif

    unsafe
    {
        /* Clock bitclock clock block from master clock pin (divided) */
        configure_clock_src_divide(clk_audio_bclk, (port) p_mclk_in, (divide/2));
        configure_port_clock_output(p_bclk, clk_audio_bclk);
    }

    if(!isnull(p_lrclk))
    {
        /* Clock LR clock from bit clock-block */
        configure_out_port_no_ready(p_lrclk, clk_audio_bclk, 0);
    }

#if (I2S_CHANS_ADC != 0)
    /* Some adustments for timing. Sample ADC lines on negative edge and add some delay */
    if(XUA_PCM_FORMAT == XUA_PCM_FORMAT_TDM)
    {
        for(int i = 0; i < numPortsAdc; i++)
        {
            set_port_sample_delay(p_i2s_adc[i]);
            set_pad_delay(p_i2s_adc[i], 4);
        }
    }
    /* ClearSound: timing alignment for the ADC1 loopback input.
     *
     * This port is clocked by clk_audio_bclk, which is derived from the MCLK
     * PIN (configure_clock_src_divide above), so its sampling instant inherits
     * the MCLK arrival time. Per xs1.h an input port samples on the clock's
     * RISING edge by default; the DAC data looped into it is launched by the
     * same clock block on its FALLING edge, as is BCLK itself. The budget from
     * launch to sample is therefore half a bit-clock period minus the output
     * pad delays of the data and clock pins, the 33 ohm series resistors, the
     * board trace and the rework jumper. At 384 kHz that budget is 20.35 ns
     * and no compensation runs at all in I2S mode (the block above is TDM
     * only), so the port samples inside the data transition and words return
     * rotated by 1-5 bits. set_pad_delay() adds 0..5 processor-clock cycles
     * (1.667 ns each at the 600 MHz tile clock) of input delay; these two
     * knobs let the sample point be measured instead of guessed. The bench
     * sweeps them with the gapless DC test. */
#ifdef CSD_ADC1_PAD_DELAY
    set_pad_delay(p_i2s_adc[0], CSD_ADC1_PAD_DELAY);
#endif
#ifdef CSD_ADC1_SAMPLE_DELAY
    set_port_sample_delay(p_i2s_adc[0]);
#endif
#endif

#elif (CODEC_MASTER)
    /* Stop bit and master clock blocks */
    stop_clock(clk_audio_bclk);

    /* Clock bclk clock-block from bclk pin */
    configure_clock_src(clk_audio_bclk, p_bclk);

    configure_in_port_no_ready(p_lrclk, clk_audio_bclk);

 	/* Do some clocking shifting to get data in the valid window */
    /* E.g. Only shift when running at 88.2+ kHz TDM slave */
    int bClkDelay_fall = 0;
    if(curSamFreq * I2S_CHANS_PER_FRAME * XUA_I2S_N_BITS >= 20000000)
    {
        /* 18 * 2ns = 36ns. This results in a -4ns (36 - 40) shift at 96KHz and -8ns (36 - 44) at 88.4KHz */
        bClkDelay_fall = 18;
    }

    set_clock_fall_delay(clk_audio_bclk, bClkDelay_fall);
#endif

#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
     /* Clock I2S/DSD output data ports from b-clock clock block */
    for(int i = 0; i < numPortsDac; i++)
    {
        configure_out_port_no_ready(p_i2s_dac[i], clk_audio_bclk, 0);
    }
#endif

#if (I2S_CHANS_ADC != 0)
    /* Clock I2S input data ports from clock block */
    for(int i = 0; i < numPortsAdc; i++)
    {
        configure_in_port_no_ready(p_i2s_adc[i], clk_audio_bclk);
    }
#endif

    /* Start clock blocks ticking */
    start_clock(clk_audio_bclk);

#endif //#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
}


void DeConfigAudioPorts(
#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
    out_buffered_port_32_t p_i2s_dac[],
    int numPortsDac,
#endif
#if (I2S_CHANS_ADC != 0)
    in_buffered_port_32_t p_i2s_adc[],
    int numPortsAdc,
#endif
#if (CODEC_MASTER == 0)
    NULLABLE_RESOURCE(out_buffered_port_32_t, p_lrclk),
    out_buffered_port_32_t p_bclk,
#else
    NULLABLE_RESOURCE(in_buffered_port_32_t, p_lrclk),
    in_buffered_port_32_t p_bclk,
#endif
    NULLABLE_RESOURCE(in_port_t,  p_mclk_in),
    clock clk_audio_bclk)
{
#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
    int tmp; 
    /* We always expect a mclk pin when xcore is master */
    assert(!isnull(p_mclk_in));

    /* Note handling of MCLK is done outside of XUA by user */

    /* Clear modes/reset port and set to Hi-Z */
    stop_port(p_bclk);
    start_port(p_bclk);
    asm volatile("in %0, res[%1]":"=r"(tmp):"r"(p_bclk):"memory");

    /* When in DSD mode p_lrclk is null, so check here (allows code reuse) */
    if(!isnull(p_lrclk))
    {
        /* Clear modes/reset port and set to Hi-Z */
        stop_port(p_lrclk);
        start_port(p_lrclk);
        asm volatile("in %0, res[%1]":"=r"(tmp):"r"(p_lrclk):"memory");
    }

#if (I2S_CHANS_DAC != 0) || (DSD_CHANS_DAC != 0)
    for(int i = 0; i < numPortsDac; i++)
    {
        /* Clear modes/reset port and set to Hi-Z */
        stop_port(p_i2s_dac[i]);
        start_port(p_i2s_dac[i]);
        asm volatile("in %0, res[%1]":"=r"(tmp):"r"(p_i2s_dac[i]):"memory");
    }
#endif

#if (I2S_CHANS_ADC != 0)
    /* Clock I2S input data ports from clock block */
    for(int i = 0; i < numPortsAdc; i++)
    {
        /* Clear modes/reset port and set to Hi-Z */
        stop_port(p_i2s_adc[i]);
        start_port(p_i2s_adc[i]);
        p_i2s_adc[i] :> int _;
    }
#endif

#endif //#if (I2S_CHANS_DAC != 0) || (I2S_CHANS_ADC != 0)
}