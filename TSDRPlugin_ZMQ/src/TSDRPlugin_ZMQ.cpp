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
#     Ted DeLoggio - zmq plugin
#-------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <zmq.hpp>
#include <gnuradio/block.h>
#include <gnuradio/io_signature.h>
#include <pmt/pmt.h>
#include <chrono>
#include <thread>

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

using namespace std::chrono_literals;
namespace po = boost::program_options;

const size_t num_buffers = 256;
int buffer_index = 0;
const size_t samples_per_buffer = 8192;
const size_t num_transfers = 32;
pthread_t rx_thread;
void **buffers;
tsdrplugin_readasync_function tsdrplugin_callback;
void *tsdrplugin_ctx;

uint32_t samp_rate = 10e6;
volatile int running = 0;
zmq::context_t *context = nullptr;
zmq::socket_t *subscriber = nullptr;

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_getName(char * name)
{
	strcpy(name, "TSDR ZeroMQ Plugin");
}

/**
 * This function will be called by libbladerf when there are samples to be 
 * processed.
 */
/*
void * stream_cb(struct bladerf *dev,
                 struct bladerf_stream *stream,
                 struct bladerf_metadata *md,
                 void* samples,
                 size_t num_samples,
                 void* user_data)
{
    static float floatbuf[samples_per_buffer * 2];
    static int16_t *data;

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
    for(int i=0; i<num_samples; i++)
    {
        //floatbuf[i] = 0.0;

        //int16_t *data = (int16_t *)buffers[buffer_index];
        data = (int16_t *)samples;

        //  Scale the samples to the range -1 to 1
        floatbuf[2 * i] = float(data[2 * i]) / 2048.0;
        floatbuf[2 * i + 1] = float(data[2 * i + 1]) / 2048.0;
    }

    //  Call the TSDRPlugin API callback with the converted samples
    tsdrplugin_callback(floatbuf, num_samples * 2, tsdrplugin_ctx, 0);
    
    //  This function must return the address of the next buffer for libbladerf
    //  to fill with stream data
    buffer_index++;
    if(buffer_index == num_buffers)
    {
        buffer_index = 0;
    }

    //void *rv = buffers[buffer_index];
    return (void *)buffers[buffer_index];
}*/

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_init(const char * params)
{
    printf("tsdrplugin_init called\n");

    context = new zmq::context_t( 1 );

    subscriber = new zmq::socket_t( *context, ZMQ_SUB );

    int major, minor, patch;
    zmq::version(&major, &minor, &patch);
    printf("ZMQ version %d.%d.%d\n", major, minor, patch);

    subscriber->set(zmq::sockopt::subscribe, "tempest");

    subscriber->connect( "tcp://localhost:50001" );
    
    RETURN_OK();
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_setsamplerate(uint32_t rate)
{
    printf("Ignoring GUI request to set sample rate\n");

	return samp_rate;
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_getsamplerate()
{
	return samp_rate;
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setbasefreq(uint32_t freq)
{
    printf("Ignoring GUI request to set base freq\n");

	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_stop(void)
{
    printf("tsdrplugin_stop called\n");

	running = 0;

	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setgain(float gain)
{
    printf("Ignoring GUI request to set gain\n");

    RETURN_OK();
}

// Copied from gr-zeromq

struct membuf : std::streambuf {
    membuf(void* b, size_t len)
    {
        char* bc = static_cast<char*>(b);
        this->setg(bc, bc, bc + len);
    }
};

#define GR_HEADER_MAGIC 0x5FF0
#define GR_HEADER_VERSION 0x01

size_t parse_tag_header(zmq::message_t& msg,
                        uint64_t& offset_out,
                        std::vector<gr::tag_t>& tags_out)
{
    membuf sb(msg.data(), msg.size());
    std::istream iss(&sb);

    size_t min_len =
        sizeof(uint16_t) + sizeof(uint8_t) + sizeof(uint64_t) + sizeof(uint64_t);
    if (msg.size() < min_len)
        throw std::runtime_error("incoming zmq msg too small to hold gr tag header!");

    uint16_t header_magic;
    uint8_t header_version;
    uint64_t rcv_ntags;

    iss.read((char*)&header_magic, sizeof(uint16_t));
    iss.read((char*)&header_version, sizeof(uint8_t));

    if (header_magic != GR_HEADER_MAGIC)
        throw std::runtime_error("gr header magic does not match!");

    if (header_version != 1)
        throw std::runtime_error("gr header version too high!");

    iss.read((char*)&offset_out, sizeof(uint64_t));
    iss.read((char*)&rcv_ntags, sizeof(uint64_t));

    for (size_t i = 0; i < rcv_ntags; i++) {
        gr::tag_t newtag;
        sb.sgetn((char*)&(newtag.offset), sizeof(uint64_t));
        newtag.key = pmt::deserialize(sb);
        newtag.value = pmt::deserialize(sb);
        newtag.srcid = pmt::deserialize(sb);
        tags_out.push_back(newtag);
    }

    return msg.size() - sb.in_avail();
}

/**
 * This function is called after the user clicks the Start button
 */
EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_readasync(tsdrplugin_readasync_function cb, void *ctx)
{
    printf("tsdrplugin_readasync called\n");

    tsdrplugin_callback = cb;
    tsdrplugin_ctx = ctx;

	running = 1;

    zmq::message_t msg;
    std::vector<gr::tag_t> tags;

    while(running) {
        zmq::pollitem_t items[] = { { static_cast<void*>(*subscriber), 0, ZMQ_POLLIN, 0 } };
        int timeout = 100; // ms
        zmq::poll(&items[0], 1, std::chrono::milliseconds{ timeout });

        if (items[0].revents & ZMQ_POLLIN) {            
            auto more = subscriber->get(zmq::sockopt::rcvmore);
            printf("more=%d\n");

            msg.rebuild();
            tags.clear();

            //  First message is the key string?
            const bool ok = bool(subscriber->recv(msg));
            if (!ok) {
                printf("ok was false?\n");
                continue;
            }

            printf("msg.size=%d\n", msg.size());

            auto is_multipart = subscriber->get(zmq::sockopt::rcvmore);
            printf("is_multipart=%d\n");

            msg.rebuild();

            const bool multi_ok = bool(subscriber->recv(msg));
            printf("multi_ok=%d\n", multi_ok);

            printf("msg.size=%d\n", msg.size());

            //uint64_t offset;
            //int i = parse_tag_header(msg, offset, tags);
            //printf("i=%d\n", i);

            const int num_items = msg.size() / 4;
            float *fp = static_cast<float *>(msg.data());

            tsdrplugin_callback(fp, num_items, tsdrplugin_ctx, 0);

        } else {
            std::this_thread::sleep_for(100us);
        }
	}

	RETURN_OK();
}

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_cleanup(void)
{
    printf("tsdrplugin_cleanup called\n");

    //  Tell the stream thread to exit
    running = 0;

    delete subscriber;
    delete context;
}
