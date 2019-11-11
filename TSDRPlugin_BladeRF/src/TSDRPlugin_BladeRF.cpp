/*
#-------------------------------------------------------------------------------
# Copyright (c) 2014 Martin Marinov.
#		2017 Henning Paul based on hackrf_transfer.c 
#			by Jared Boone and Benjamin Vernoux
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the GNU Public License v3.0
# which accompanies this distribution, and is available at
# http://www.gnu.org/licenses/gpl.html
#
# Contributors:
#     Martin Marinov - initial API and implementation
#     Ted DeLoggio - BladeRF plugin
#-------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libbladeRF.h>
#include <pthread.h>

#include <boost/program_options.hpp>
#include <boost/format.hpp>
#include <boost/thread.hpp>

#include <iostream>
#include <complex>

#include "TSDRPlugin.h"

#include "TSDRCodes.h"

#include <stdint.h>
#include <boost/algorithm/string.hpp>

#include "errors.hpp"

namespace po = boost::program_options;

bladerf* device = NULL;
bladerf_devinfo dev_info;
bladerf_channel channel = BLADERF_CHANNEL_RX(0);
struct bladerf_stream* rx_stream;
const size_t num_buffers = 24;
int buffer_index = 0;
const size_t samples_per_buffer = 8192;
const size_t num_transfers = 8;
pthread_t rx_thread;
void **buffers;
tsdrplugin_readasync_function tsdrplugin_callback;
void *tsdrplugin_ctx;

uint32_t samp_rate = 10e6;
volatile int running = 0;

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_getName(char * name)
{
	strcpy(name, "TSDR BladeRF Plugin");
}

/**
 * This thread exists to house the blocking call to bladerf_stream
 * Maybe there's a better way to do this?
 */
void * rx_thread_fn(void *arg)
{
    printf("rx_thread_fn starting\n");

    //  Start the bladerf stream, this will block until the stream is stopped
    int status = bladerf_stream(rx_stream, BLADERF_RX_X1);
    printf("status=%d\n", status);

    printf("rx_thread_fn stopping\n");
    return NULL;
}

/**
 * This function will be called by libbladerf when there are samples to be 
 * processed.
 */
void * stream_cb(struct bladerf *dev,
                 struct bladerf_stream *stream,
                 struct bladerf_metadata *md,
                 void* samples,
                 size_t num_samples,
                 void* user_data)
{
    static float floatbuf[samples_per_buffer];

    if(!running)
    {
        //  tsdrplugin_stop has been called, shutdown the stream
        return BLADERF_STREAM_SHUTDOWN;
    }

    //  The I/Q samples from libbladerf are 16-bit signed integers with a range
    //  of -2048 to 2048.
    //
    //  See: https://nuand.com/libbladeRF-doc/v2.2.1/group___s_t_r_e_a_m_i_n_g___f_o_r_m_a_t.html
    //
    //  These samples need to be converted to floats for the TSDRPlugin API
    for(int i=0; i<samples_per_buffer; i++)
    {
        //floatbuf[i] = 0.0;

        int16_t *data = (int16_t *)buffers[buffer_index];

        //  Scale the samples to the range -1 to 1
        floatbuf[i] = (float)data[i] / 2048.0;
    }

    //  Call the TSDRPlugin API callback with the converted samples
    tsdrplugin_callback(floatbuf, samples_per_buffer, tsdrplugin_ctx, 0);
    
    //  This function must return the address of the next buffer for libbladerf
    //  to fill with stream data
    buffer_index++;
    if(buffer_index == num_buffers)
    {
        buffer_index = 0;
    }

    //void *rv = buffers[buffer_index];
    return (void *)buffers[buffer_index];
}

/**
 * This function is called after the user clicks "Load BladeRF"
 */
EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_init(const char * params)
{
    printf("tsdrplugin_init called\n");

    //  By using an empty device_identifier string, we're telling the bladerf
    //  API to open the first device discovered
    int status = bladerf_open(&device, "");
    if(status != 0)
    {
        fprintf(stderr, "Failed to configure RX sync interface: %s\n",
                bladerf_strerror(status));
        bladerf_close(device);
        RETURN_EXCEPTION("bladerf_open() failed", TSDR_CANNOT_OPEN_DEVICE);
    }
   
    const unsigned int num_buffers   = 16;
    const unsigned int buffer_size   = 8192; /* Must be a multiple of 1024 */
    const unsigned int num_transfers = 8;
    const unsigned int timeout_ms    = 3500;
   
    status = bladerf_sync_config(device,
            BLADERF_RX_X1,
            BLADERF_FORMAT_SC16_Q11,
            num_buffers,
            buffer_size,
            num_transfers,
            timeout_ms);
    if(status != 0)
    {
        fprintf(stderr, "Failed to configure RX sync interface: %s\n",
                bladerf_strerror(status));
        bladerf_close(device);
        RETURN_EXCEPTION("bladerf_sync_config() failed", TSDR_CANNOT_OPEN_DEVICE);
    }

    status = bladerf_enable_module(device, BLADERF_RX_X1, true);
    if(status != 0)
    {
        fprintf(stderr, "Failed to enable RX: %s\n", bladerf_strerror(status));
        RETURN_EXCEPTION("bladerf_enable_module() failed", TSDR_CANNOT_OPEN_DEVICE);
    }
    
    //TODO: Use program options as the HackRF plugin did
    /*
    //setup the program options
	po::options_description desc("Allowed options");
	desc.add_options()
	("sernum", po::value<std::string>(&serial_number)->default_value(""), "HackRF device address args")
	("rate", po::value<uint32_t>(&samp_rate)->default_value(samp_rate), "rate of incoming samples")
	("amp", po::value<bool>(&amp_on), "enable input amplifier")
	("bw", po::value<uint32_t>(&bw), "daughterboard IF filter bandwidth in Hz");

	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);
	} catch (std::exception const&  ex)
	{
		std::string msg(boost::str(boost::format("Error: %s\n\nTSDRPlugin_BladeRF %s") % ex.what() % desc));
		RETURN_EXCEPTION(msg.c_str(), TSDR_PLUGIN_PARAMETERS_WRONG);
	}
    */

	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_setsamplerate(uint32_t rate)
{
    printf("GUI requested sample rate: %d\n", rate);

    samp_rate = rate;

    int status = bladerf_set_sample_rate(device, channel, samp_rate, NULL);
    if(status != 0)
    {
        fprintf(stderr, "Failed to set sample rate to %u Hz: %s\n",
                samp_rate, bladerf_strerror(status));
		RETURN_EXCEPTION("bladerf_set_sample_rate() failed", TSDR_CANNOT_OPEN_DEVICE);
    }

	return samp_rate;
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_getsamplerate()
{
	return samp_rate;
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setbasefreq(uint32_t freq)
{
    printf("GUI requested frequency: %d\n", freq);

    int status = bladerf_set_frequency(device, channel, freq);
    if(status != 0)
    {
        fprintf(stderr, "Failed to set frequency to %u Hz: %s\n",
                freq, bladerf_strerror(status));
        return status;
    }

	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_stop(void)
{
    printf("tsdrplugin_stop called\n");

	running = 0;
    pthread_join(rx_thread, NULL);
	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setgain(float gain)
{
    //  Max gain is 60dB for receive channels
    int bladerf_gain = gain * 60.0;
    printf("GUI requested gain: %f, adjusted gain: %d\n", gain, bladerf_gain);

    int status = bladerf_set_gain(device, channel, bladerf_gain);
    if (status != 0) {
        fprintf(stderr, "Failed to set gain: %s\n", bladerf_strerror(status));
		RETURN_EXCEPTION("bladerf_set_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
    }

    RETURN_OK();
}

/**
 * This function is called after the user clicks the Start button
 */
EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_readasync(tsdrplugin_readasync_function cb, void *ctx)
{
    printf("tsdrplugin_readasync called\n");

    tsdrplugin_callback = cb;
    tsdrplugin_ctx = ctx;

    //  Initialize a stream for bladerf asynchronous API
    int status = bladerf_init_stream(
            &rx_stream,
            device,
            stream_cb,
            &buffers,
            num_buffers,
            BLADERF_FORMAT_SC16_Q11,
            samples_per_buffer,
            num_transfers,
            NULL);
    if(status != 0)
    {
        fprintf(stderr, "Failed to initialize stream: %s\n", bladerf_strerror(status));
		RETURN_EXCEPTION("bladerf_stream_init() failed", TSDR_CANNOT_OPEN_DEVICE);
    }

	running = 1;

    //  The call to bladerf_stream will block as long as the stream is running
    //  so we'll call it in another thread.
    status = pthread_create(&rx_thread, NULL, rx_thread_fn, NULL);
    if(status)
    {
        fprintf(stderr, "Failed to create rx thread\n");
		RETURN_EXCEPTION("pthread_create() failed", TSDR_CANNOT_OPEN_DEVICE);
    }

    while(running){
        //  The application seems to want this thread to spin...
        sleep(0);
	}

	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_cleanup(void)
{
    printf("tsdrplugin_cleanup called\n");

    //  Tell the stream thread to exit
    running = 0;

    bladerf_close(device);
}
