/*
#-------------------------------------------------------------------------------
# Copyright (c) 2014 Martin Marinov.
#		2018 Tan Peng Chong based on airspy-tools/src/airspy_rx.c 
#			by Jared Boone and Benjamin Vernoux
# All rights reserved. This program and the accompanying materials
# are made available under the terms of the GNU Public License v3.0
# which accompanies this distribution, and is available at
# http://www.gnu.org/licenses/gpl.html
#
# Contributors:
#     Martin Marinov - initial API and implementation
#     Tan Peng Chong - rudimentary native support for Airspy
#-------------------------------------------------------------------------------
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <libairspy/airspy.h>

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

// When using FLOAT32 REAL
#define BUFFER_SIZE 131072


// When using FLOAT32 IQ
//#define BUFFER_SIZE 65536


#define DEFAULT_VGA_IF_GAIN (7)
#define DEFAULT_LNA_GAIN (7)
#define DEFAULT_MIXER_GAIN (7)



namespace po = boost::program_options;

typedef struct {
	tsdrplugin_readasync_function cb;
	void *ctx;
	float *floatbuff;
	size_t buff_size;
} cb_ctx_t;


receiver_mode_t receiver_mode = RECEIVER_MODE_RX;


uint32_t vga_gain = DEFAULT_VGA_IF_GAIN;
uint32_t lna_gain = DEFAULT_LNA_GAIN;
uint32_t mixer_gain = DEFAULT_MIXER_GAIN;

uint32_t linearity_gain_val = 0;
bool linearity_gain = false;

uint32_t sensitivity_gain_val = 0;
bool sensitivity_gain = false;


bool serial_number = false;
uint64_t serial_number_val = 0;

//struct airspy_device* device = NULL;
static airspy_device* device = NULL;

enum airspy_sample_type sample_type_val = AIRSPY_SAMPLE_FLOAT32_REAL;
//enum airspy_sample_type sample_type_val = AIRSPY_SAMPLE_FLOAT32_IQ;
//enum airspy_sample_type sample_type_val = AIRSPY_SAMPLE_INT16_IQ;
// AIRSPY_SAMPLE_FLOAT32_IQ:
// AIRSPY_SAMPLE_FLOAT32_REAL:
// AIRSPY_SAMPLE_INT16_IQ:
// AIRSPY_SAMPLE_INT16_REAL:
// AIRSPY_SAMPLE_UINT16_REAL:
// AIRSPY_SAMPLE_RAW:

uint32_t req_freq = 105e6;


// TODO: Verify to use sample_rate_val or req_rate
// Does the GUI allows specifying the rate?
uint32_t sample_rate_val = 10000000;
uint32_t req_rate = 10e6;
uint32_t sample_rate_mini = 6000000;
uint32_t req_rate_mini = 6e6;


float req_gain = 1;
volatile int is_running = 0;



EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_getName(char * name) {
	strcpy(name, "TSDR Airspy Compatible Plugin");
}

int rx_callback(airspy_transfer* transfer) {

	if(is_running){
		cb_ctx_t *airspy_callback_ctx = (cb_ctx_t*)transfer->ctx;
		size_t buff_size=airspy_callback_ctx->buff_size;
		float *floatbuff=airspy_callback_ctx->floatbuff;
		void *ctx=airspy_callback_ctx->ctx;
		tsdrplugin_readasync_function cb=airspy_callback_ctx->cb;

		//fprintf(stderr, "buff_size=%d, floatbuff=%x, ctx=%x, cb=%x\n",buff_size,floatbuff,ctx,cb);
		//fprintf(stderr, "buff_size=%d, floatbuff=%x, ctx=%x, cb=%x, sample_count=%d\n",buff_size,floatbuff,ctx,cb, transfer->sample_count);

		assert(transfer->sample_count<=BUFFER_SIZE);


		// TODO: Why sample value divide by 128?
		if (transfer->sample_count==buff_size){
			for(int i=0;i<transfer->sample_count;i++){
				//floatbuff[i]=(float)((transfer->samples)[i])/128.0;
				//floatbuff[i]=(float)(transfer->samples)[i];
				floatbuff[i]=((float *)(transfer->samples))[i];
				floatbuff[i]=floatbuff[i]/128.0;
			}
			cb(floatbuff, transfer->sample_count, ctx, 0);

		}
		else{
			cb(floatbuff, 0, ctx, transfer->sample_count);
			int result = airspy_stop_rx(device);
			is_running=0;
		}

	}
	return 0;
}


EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_init(const char * params) {
	int result;

	// simulate argv and argc
	std::string sparams(params);

	typedef std::vector< std::string > split_vector_type;

	split_vector_type argscounter;
	boost::split( argscounter, sparams, boost::is_any_of(" "), boost::token_compress_on );

	const int argc = argscounter.size()+1;
	char ** argv = (char **) malloc(argc*sizeof(char *));
	char zerothtarg[] = "TSDRPlugin_Airspy";
	argv[0] = (char *) zerothtarg;
	for (int i = 0; i < argc-1; i++)
	argv[i+1] = (char *) argscounter[i].c_str();

	//variables to be set by po
	std::string serial_number, file;
	uint32_t bw;


	// TODO: Support user inputs for VGA, MIXER, LNA, Linearity, sensitivity gains
	// TODO: Validate the inputs of these parameters is within range, etc.
	//setup the program options
	po::options_description desc("Allowed options");
	desc.add_options()
	("serial", po::value<uint64_t>(&serial_number_val), "Airspy device address args")
	("rate", po::value<uint32_t>(&sample_rate_val)->default_value(sample_rate_val), "rate of incoming samples")
	("vga_gain", po::value<uint32_t>(&vga_gain)->default_value(vga_gain), "vga gain value")
	("lna_gain", po::value<uint32_t>(&lna_gain)->default_value(lna_gain), "lna gain value")
	("mixer_gain", po::value<uint32_t>(&mixer_gain)->default_value(mixer_gain), "mixer gain value")
	("linearity_gain", po::value<uint32_t>(&linearity_gain_val), "linearity gain value")
	("sensitivity_gain", po::value<uint32_t>(&sensitivity_gain_val), "sensitivity gain value");


	po::variables_map vm;
	try {
		po::store(po::parse_command_line(argc, argv, desc), vm);
		po::notify(vm);
	} catch (std::exception const&  ex)
	{
		std::string msg(boost::str(boost::format("Error: %s\n\nTSDRPlugin_Airspy %s") % ex.what() % desc));
		RETURN_EXCEPTION(msg.c_str(), TSDR_PLUGIN_PARAMETERS_WRONG);
	}


	receiver_mode = RECEIVER_MODE_RX;

	result = airspy_init();
	if( result != AIRSPY_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("airspy_init() failed", TSDR_CANNOT_OPEN_DEVICE);
	}


	if(serial_number_val!=0)
	{
		result = airspy_open_sn(&device, serial_number_val);
		if( result != AIRSPY_SUCCESS ) {
			free(argv);
			RETURN_EXCEPTION("airspy_open_sn() failed", TSDR_CANNOT_OPEN_DEVICE);
		}
	}else
	{
		result = airspy_open(&device);
		if( result != AIRSPY_SUCCESS ) {
			free(argv);
			RETURN_EXCEPTION("airspy_open() failed", TSDR_CANNOT_OPEN_DEVICE);
		}
	}


	result = airspy_set_sample_type(device, sample_type_val);
	if (result != AIRSPY_SUCCESS) {
		free(argv);
		RETURN_EXCEPTION("airspy_set_sample_type() failed", TSDR_CANNOT_OPEN_DEVICE);
	}


	// TODO: To validate the user input values of sample rate
	//airspy_get_samplerates(device, &count, 0);
	//supported_samplerates = (uint32_t *) malloc(count * sizeof(uint32_t));
	//airspy_get_samplerates(device, supported_samplerates, count);

	result = airspy_set_samplerate(device, sample_rate_val);
	if (result != AIRSPY_SUCCESS) {
		result = airspy_set_samplerate(device, sample_rate_mini);
		if (result != AIRSPY_SUCCESS) {
			free(argv);
			RETURN_EXCEPTION("airspy_set_samplerate() failed", TSDR_CANNOT_OPEN_DEVICE);
		}
	}


	// TODO: To validate the user input values of vga,mixer,lna gain
	result = airspy_set_vga_gain(device, vga_gain);
	if( result != AIRSPY_SUCCESS ) {
			free(argv);
			RETURN_EXCEPTION("airspy_set_vga_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = airspy_set_mixer_gain(device, mixer_gain);
	if( result != AIRSPY_SUCCESS ) {
			free(argv);
			RETURN_EXCEPTION("airspy_set_mixer_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	result = airspy_set_lna_gain(device, lna_gain);
	if( result != AIRSPY_SUCCESS ) {
			free(argv);
			RETURN_EXCEPTION("airspy_set_lna_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	}


	// TODO: Set Linearity, Sensitivity

	/*
	if( linearity_gain == true )
	{
		result =  airspy_set_linearity_gain(device, linearity_gain_val);
		if( result != AIRSPY_SUCCESS ) {
			fprintf(stderr, "airspy_set_linearity_gain() failed: %s (%d)\n", airspy_error_name(result), result);
		}
	}

	if( sensitivity_gain == true )
	{
		result =  airspy_set_sensitivity_gain(device, sensitivity_gain_val);
		if( result != AIRSPY_SUCCESS ) {
			fprintf(stderr, "airspy_set_sensitivity_gain() failed: %s (%d)\n", airspy_error_name(result), result);
		}
	}
	*/

	result = airspy_set_freq(device, req_freq);
	if( result != AIRSPY_SUCCESS ) {
		free(argv);
		RETURN_EXCEPTION("airspy_set_freq() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	fprintf(stderr, "init succeeded\n");

	free(argv);
	RETURN_OK();

	return 0; // to avoid getting warning from Eclipse
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_setsamplerate(uint32_t rate) {
	if (is_running)
	return tsdrplugin_getsamplerate();

	req_rate = rate;

	int result = airspy_set_samplerate(device, req_rate);
	if( result != AIRSPY_SUCCESS ) {
		RETURN_EXCEPTION("airspy_set_samplerate() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	return req_rate;
}

EXTERNC TSDRPLUGIN_API uint32_t __stdcall tsdrplugin_getsamplerate() {

	return req_rate;
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setbasefreq(uint32_t freq) {

	req_freq = freq;

	int result = airspy_set_freq(device, req_freq);
	if( result != AIRSPY_SUCCESS ) {
		RETURN_EXCEPTION("airspy_set_freq() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	RETURN_OK();

	return 0; // to avoid getting warning from Eclipse
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_stop(void) {
	is_running = 0;
	RETURN_OK();

	return 0; // to avoid getting warning from Eclipse
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_setgain(float gain) {

	// Airspy gain is from 0 to 14

	int req_lna_gain=int((gain*14.0)+0.5);	
	
	
	// Airspy sensitivity gain is from 0 to 15
	int req_sensitivity_gain=int((gain*15.0)+0.5);	

	//fprintf(stderr, "requested gain %f, set gain %d\n",gain,req_lna_gain);

	//int result = airspy_set_lna_gain(device, req_lna_gain);
	//if( result != AIRSPY_SUCCESS ) {
	//	RETURN_EXCEPTION("airspy_set_lna_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
	//}


        int result = airspy_set_sensitivity_gain(device, req_sensitivity_gain);
        
	if( result != AIRSPY_SUCCESS ) {
		RETURN_EXCEPTION("airspy_set_sensitivity_gain() failed", TSDR_CANNOT_OPEN_DEVICE);
        }



	return 0; // to avoid getting warning from Eclipse
}

EXTERNC TSDRPLUGIN_API int __stdcall tsdrplugin_readasync(tsdrplugin_readasync_function cb, void *ctx) {
	cb_ctx_t airspy_callback_ctx;
	float *floatbuff;

	is_running = 1;

	float * buff = NULL;

	fprintf(stderr, "entered tsdrplugin_readasync()\n");

	int result = airspy_set_samplerate(device, req_rate);
	if( result != AIRSPY_SUCCESS ) {
		int result = airspy_set_samplerate(device, req_rate_mini);
		if( result != AIRSPY_SUCCESS ) {
			RETURN_EXCEPTION("airspy_set_samplerate() failed", TSDR_CANNOT_OPEN_DEVICE);
		}
	}

	result = airspy_set_vga_gain(device, vga_gain);
	result |= airspy_set_lna_gain(device, lna_gain);

	result = airspy_set_freq(device, req_freq);
	if( result != AIRSPY_SUCCESS ) {
		RETURN_EXCEPTION("airspy_set_freq() failed", TSDR_CANNOT_OPEN_DEVICE);
	}


	size_t buff_size = BUFFER_SIZE;
	floatbuff = (float *)malloc(buff_size * sizeof(float));

	airspy_callback_ctx.buff_size = buff_size;
	airspy_callback_ctx.floatbuff = floatbuff;
	airspy_callback_ctx.ctx=ctx;
	airspy_callback_ctx.cb=cb;

	//fprintf(stderr, "&airspy_callback_ctx=%x, buff_size=%d, floatbuff=%x, ctx=%x, cb=%x\n",&airspy_callback_ctx,buff_size,floatbuff,ctx,cb);

	
	result = airspy_start_rx(device, (airspy_sample_block_cb_fn)rx_callback, &airspy_callback_ctx);
	if( result != AIRSPY_SUCCESS ) {
		RETURN_EXCEPTION("airspy_start_rx() failed", TSDR_CANNOT_OPEN_DEVICE);
	}

	while(is_running && (airspy_is_streaming(device) == AIRSPY_TRUE)){

	} done_loop:

	result = airspy_stop_rx(device);
	if( result != AIRSPY_SUCCESS ) {
		if (buff!=NULL) free(buff);
		RETURN_EXCEPTION("airspy_stop_rx() failed", TSDR_CANNOT_OPEN_DEVICE);
	} else {
		fprintf(stderr, "airspy_stop_rx() done\n");
	}


	if (floatbuff!=NULL) free(floatbuff);

	RETURN_OK();

	return 0; // to avoid getting warning from Eclipse
}

EXTERNC TSDRPLUGIN_API void __stdcall tsdrplugin_cleanup(void) {

	int result = airspy_close(device);
	if(result != AIRSPY_SUCCESS) {
		fprintf(stderr, "airspy_close() failed\n");
	} else {
		fprintf(stderr, "airspy_close() done\n");
	}

	airspy_exit();
	fprintf(stderr, "airspy_exit() done\n");

	is_running = 0;
}
