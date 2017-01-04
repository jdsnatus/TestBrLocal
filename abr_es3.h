/*!
 * \file teoae_es3.h
 * \author  Robert DeCarlo
 * \date 2012-04-20
 * \version 1.0
 *
 * \copyright Copyright (c) 2012 Natus Medical Incorporated.
 * All rights reserved.
 *
 * This software is the confidential and proprietary information of Natus
 * Medical Incorporated. ("Confidential Information").
 */
#ifndef NATUS_ABR_ES3_H
#define NATUS_ABR_ES3_H

#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include "buffer.h"



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
//
//		ABR constants
//
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//#define ES_QBLEN  (192)
#define ES_QBLEN  (180)			// Redefinition
//#define ABR_RMS_TC  (0x78d7)	// Redefinition
#define ABR_TC  (0x0729)
/*
#define ES_ESTART (96)
#define ES_ESTOP (192)
#define TOAE_SQRT_ADD (4)
#define SQRT_FACT (10)
#define ABR_FRAMELEN  (180)
#define ABR_OFFSET  (8)
#define ABR_N_SHIFT  (100)
#define ABR_N_SHIFT_2  (50)
#define ABR_SHIFT_ADD  (72)
#define BINOM_PREVIEW  (44)
#define ABR_DC_START  (25)
#define ABR_BINOM_INTERFERENCE  (256)
#define N_Q  (8)
#define ABR_WHEN_CRIT  (15)
#define ABR_CRITERION (0x0730)
#define ABR_OFFSET_SENS	(0x0800)
#define ABR_TC  (0x0729)
#define ABR_SQRT_ADD  (0x0200)
#define ABR_SQRT_FACT  (0x0010)
#define ABR_ESTART  (40)
#define ABR_ESTOP  (ABR_N_SHIFT)
#define ONE_HALF_Q15 (0x4000)
#define ABR_DC_MAX_THRES (32767) //FIXME
#define SQRT_RESOLUTION_ENH (7200) //empirically fitted to old version, approx.

*/
#define	ABR_FS	 			  10200 	    							// sampling rate
#define	ABR_N_SHIFT 		  100 									// number of shifts
#define	ABR_OFFSET			  8
#define	ABR_DC_START		  25 										// where in frame to start DC compensation (1..ABR_ESTART)
#define	ABR_ESTART 			  40 								   	// start of evaluation region
#define	ABR_ESTOP 			  ABR_N_SHIFT 		  			 		// end of evaluation region
#define	ABR_JITTERMAX 		  16 				   					// max jitter samples		   (10)
#define	ABR_LONG_JITTER	  40 									   // max extra jitter samples (50)
#define	ABR_WHEN_CRIT 		  15 										// calc the criterion every x+1 frames
#define	ABR_WHEN_JITT 		  31 										// calc the criterion every x+1 frames
#define	ABR_WHEN_SHOW_CRIT	  63 									// show the criterion every x+1 frames
#define	ABR_CRIT_BYTE 		  115 									// sqrt(60)*8   62 equals 1. Factor 16/15 applies// AL 15.7.03
#define	ABR_CRIT_SHIFT		  4 	 									// resolution enhancement
#define	ABR_CRIT_FACT		  16 									   // 2 exp ABR_CRIT_SHIFT
#define	ABR_CRITERION 		  ABR_CRIT_BYTE*ABR_CRIT_FACT
#define	ABR_CRIT_SCALE 	  ABR_CRIT_BYTE*ABR_CRIT_BYTE/(72+24)    	// scale for plotting the criterion
#define	ABR_SQRT_ADD 		  512   									// safety-add-on for sqrt to prevent early false passes

#define	ABR_PLOT_START		  40 				  						// where to start plotting in curve data
#define	ABR_FRAMELEN 		  ABR_N_SHIFT+73-1+ABR_OFFSET 	// frame length
#define	ABR_PLOT_LEN		  ABR_N_SHIFT - ABR_PLOT_START - 1  //59 < > 60
#define	ABR_Y_SCALE			  7 										// height of crit marker in pixel
#ifdef ENABLE_PLOT_CACHE
#define 	ABR_WHEN_PLOT 		  63 										// plot every x+1 frames
#else
#define 	ABR_WHEN_PLOT 		  127 									// plot every x+1 frames
#endif
#define 	ABR_WHEN_PLOT_A	  3 										// offset
#define	ABR_WHEN_KEY		  3 									   // test keyboard every n+1 frames
#define	ABR_1ST_CRIT		  512 									// 1st calculation of criterion after x frames
#define	ABR_WHEN_FRAMES 	  63  									// display time bar every x+1 frames
#define	ABR_WHEN_NOISE 	  7 								   	// display noise every x+1 frames
#define	ABR_WHEN_IMP 		  1023 									// impedance measurement every x+1 frames
#define	ABR_WHEN_LED		  15
#define	ABR_WHEN_STIM 		  15 										// when to test stimulus amplitude
#define	ABR_CAL_WHEN_STIM	  31 										// stimulus control in calibration
#define	ABR_CAL_WHEN_PLOT	  7
#define	ABR_HIST_SCALE 	  256 								   // scale for plotting the histogramm
#define	ABR_MAX_TIME 		  45000 									// max number of frames before 'refer'. factor 3.5 applies for low EEG (makes 13k frames)
#define	ABR_MAX_TIME2 		  22000 									// number of frames for fast refer
#define	ABR_TIME_SCALE		  72 * 32768 / ABR_MAX_TIME
#define	ABR_TIME_SCALE_C	  100 * 32768 / ABR_MAX_TIME
#define	ABR_STIM_TC			  8192
#define	ABR_STIM_CONTROL_START   28 								// start of stimulus control region
#define	ABR_STIM_CONTROL_STOP   55 								// end of stimulus control region
#define	ABR_CHECK_NOISE_START   58 								// where to start looking for ambient noise
#define	ABR_STIM_CONTROL_FACT   4
#ifdef PROBE_PROM
#ifdef ABR_ADD_3_DB
#define	ABR_STIM_LEVEL		  1060 									// AL 31.7.03 for release 1.06
#define  	ABR_MIN_VOL 		  28 					   				// min volume for stimulus
#define  	ABR_MAX_VOL 		   7 									   // max volume for stimulus //////////////// 11 to 10 16.1.04 AL
#else
#define	ABR_STIM_LEVEL		  750 									// AL 31.7.03 for release 1.06
#define  	ABR_MIN_VOL 		  30 					   				// min volume for stimulus
#define  	ABR_MAX_VOL 		   9 									   // max volume for stimulus //////////////// 11 to 10 16.1.04 AL
#endif

#else
//#define	ABR_STIM_LEVEL		  600 									// desired stimulus feedback level
#define	ABR_STIM_LEVEL		  750 									// AL 31.7.03 for release 1.06
#define  	ABR_MIN_VOL 		  23 					   				// min volume for stimulus
#define  	ABR_MAX_VOL 		  4 									   // max volume for stimulus
#endif
//#define	ABR_SPK_VOL 		  0x0e0e
#define	ABR_SPK_VOL 		  0x0e0e   								// AL 17.6.03 for release 1.04 //////////// AL from 0d0d to 0e0e 16.1.04
#define	ABR_SPK_VOL_5		  ABR_SPK_VOL-0x0303 				//  5 dB stronger
#define	ABR_SPK_VOL_10		  ABR_SPK_VOL-0x0707 				// 10 dB stronger
#define	ABR_SPK_VOL_15		  ABR_SPK_VOL-0x0a0a 				// 15 dB stronger  // max with TE probe
#define	ABR_SPK_VOL_20		  ABR_SPK_VOL-0x0d0d 				// 20 dBs stronger // this would work with DP probe only

#ifdef ABR_ADD_3_DB
#define	ABR_Y_VOL			  0x0a0a 								// nominal y-cable level
#define	ABR_Y_VOL_5 		  0x0707 								//  5 dBs higher
#define	ABR_Y_VOL_10		  0x0303 								// 10 dBs higher
#define	ABR_Y_VOL_15		  0x0000 								// 15 dBs higher
#else
#define	ABR_Y_VOL			  0x0c0c 								// nominal y-cable level
#define	ABR_Y_VOL_5 		  0x0909 								//  5 dBs higher
#define	ABR_Y_VOL_10		  0x0505 								// 10 dBs higher
#define	ABR_Y_VOL_15		  0x0202 								// 15 dBs higher
#endif

#define	ABR_STIM_LO 		  ABR_STIM_LEVEL *  50 / 100 		// lower limit during measurement
#define	ABR_CAL_LO			  ABR_STIM_LEVEL *  83 / 100 		// lower limit for calibration
#define	ABR_CAL_HI			  ABR_STIM_LEVEL * 120 / 100 		// upper limit for calibration
#define	ABR_STIM_HI 		  ABR_STIM_LEVEL * 200 / 100 		// upper limit during measurement
//#define	ABR_COUPLER_STIM_HI 		  ABR_STIM_LEVEL * 10 		// upper limit during measurement	w/ coupler
#define	ABR_CAL_SCALE 		  72 * 16384 / ABR_STIM_LEVEL 	// scale for calibration bar
#define	ABR_VOL_SCALE		  ABR_MIN_VOL-ABR_MAX_VOL
#define	ABR_STABLE_UNTIL_GREEN   32 	 							// stable frames necessary to start measurement
#define	ABR_STABLE_UNTIL_CONT   100 	 							// stable frames necessary to start measurement
#define	ABR_NOISE_TC		  4096
#define	ABR_CAL_MAXFRAMES   8000
#define	IMP_STIM_FRAME 	  ABR_FRAMELEN+ABR_JITTERMAX/2 	// avg frame len for continuing the stimulus during imp test
#define	AABR_RES_LEN 		  154 									//
#define 	ABR_SIGMA_SCALE 	  8 										// plot size of 3 sigma
#define 	ABR_PLOT_SCALE 	  4 										// plot size of 3 sigma
//#define	ABR_OFFSET_SENS	  0x3000 								// sensitivity of offset-correction in ABR
#define	ABR_OFFSET_SENS	  0x800 									// sensitivity of offset-correction in ABR
#define 	ABR_SIGMA_PLOT 	  8 										// plot size of 3 sigma
#define	ABR_SQRT_FACT 		  16

#define	ABR_BINOM_INTERFERENCE   256  			// counter end to go into interference mode
#define  	ABR_BINOM_ADD   4 							// this many steps forth, 1 step back...
#define  	ABR_BINOM_ADD2   5 							// hysteresis
#define	BINOM_PREVIEW   44 							// asymmetry to accept. 20 equals sigma
#define  	ABR_BINOM_SKIP   2 							// number of frames to skip after binom preview crit

#define	ABR_CLIP_LIMIT   20000 					   // limit for clip detection. some extra safety included...
//#define NOTCH_OVER		   8192 						// limit the compensation buffer amplitude
#define 	NOTCH_OVER		  16384 						// limit the compensation buffer amplitude (Australia)
#define 	VNOTCH_OVER		  8192 						// limit the compensation buffer amplitude of vnotch filter

#define	ABR_DC_MAX_THRES   4096 					// acceptable abr_dc_max from hipass filter

//matlab: fix((1-10^(  -1 /20))*32768)
//                    ^^^
//#define	ABR_RMS_TC			  3563 					// -1 dB
#define	ABR_RMS_TC			  1833 					// -0.5 dB

#define	ABR_PAUSE_FRAMES    300*60 				// appr. 5 minutes
#define 	ABR_NOISE_TRES_Y	   30 					// AL 6.10.03: old: 90
#define 	ABR_NOISE_TRES		  2000 					// "too much noise" threshold. rised: 17.12.03 AL
#define	ABR_SKIP_AFTER_LOUD   3 					// number of frames to skip after a 'loud' one
#define	ABR_REC_GAIN		  2
#define	ABR_CAL_WHEN_SHOW_MSG   512



// These values set the specifics of sizes and rates uses
#define ABR_INPUT_SR (48000)            // input Sample rate
#define ABR_OUTPUT_SR (10200)           // internal Sample rate
#define ABR_NUMBER_SAMPLES (ABR_FRAMELEN)  // number of contiguous click samples
//#define ABR_DECIMATION_FACTOR (INPUT_SR/OUTPUT_SR) // sample rate conversion
#define ABR_DECIMATION_FACTOR (4.7) // sample rate conversion



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// 	Impedance measurement
//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define 	ABR_IMP_FRAMELEN 	  816
#define 	ABR_IMP_CYCLE_LEN 	  48
#define     ABR_IMP_MAX_IMP       990  // bug 3414 max imp value check


//#define	IMP_FS			  10200 						// sampling rate for imp test
#define 	IMP_FRAMELEN 	  10 							// 50 samples at fs   10kHz: 200 Hz
#define	IMP_FRAMELEN_2   IMP_FRAMELEN / 2 		// half frame len
#define	IMP_CAL_OFFSET	  43 							// calibration offset (compensation for 2*1k internal resistors)
#define	IMP_CAL_FACT	  15100 					   // calibration factor
#define	IMP_SHIFT 		  -5 							// shift bits to compensate averaging
#define	IMP_AVG 			  64 							// averages ( periods) for single imp measurement
#define	IMP_SKIP_FRAMES   64 						// frames to skip before measuring impedance
#define	IMP_POS1 		  1 							// switching positions for the sense signal

#define	IMP_POS2 		  IMP_FRAMELEN_2+1 		// switching positions for the sense signal


#define	ABR_IMP_GOOD	  60 					    	// treshold for 'good' impedance in 100 Ohms
#define	ABR_IMP_FAIR	  120 						// treshold for 'fair' impedance
#define	ABR_IMP_OK 		  120 					   // treshold for impedance that is still accepted
#define	ABR_IMP_OK1		  130 					   // treshold for impedance within measurement
#define	ABR_SYM_GOOD	  20 							// treshold for good balance
#define	ABR_SYM_FAIR	  40 						   // treshold for fair balance
#define	ABR_SYM_OK 		  40 							// treshold for accepted balance
#define	ABR_SYM_OK1		  45 							// treshold for balance within measurement

#define	IMP_MAX_FRAMES   240 						// limit impedance measurement time to save relais lives//

//#define ABR_NUMBER_SAMPLES (192)		// Redefinition
#define ABR_NUMBER_QBUFS (8)
#define ABR_FRAME_SIZE (180)
#define ABR_STIM_LENGTH (846)

static const int32_t abr_es_weights[ABR_NUMBER_QBUFS] =
    { 0, 0, 0, 0, 8192, 16384, 23170, 32767 };

// Structure used for reporting status to the user interface

typedef struct vnotch_tag
{
    int32_t vnotch_delay;
    int32_t vnotch_1;
    int32_t vnotch_2;
    int32_t vnotch_3;
    int32_t vnotch_cntr;
    int32_t vnotch_mody_cntr;
    uint32_t vnotch_used_len;
    buffer vnotch_buf;
    buffer vnotch_dbuf;
} notch_state_t;

typedef struct abr_status_tag
{
    int32_t Percent_Progress_Time;
    int32_t Percent_Progress_ABR;
    int32_t Percent_EEG_Level;
    int32_t Percent_Noise_Indicator;
    int16_t Waveform[ABR_FRAME_SIZE];
    int32_t Upper_Threshold;
    int32_t Lower_Threshold;
} abr_status_t;

// Structure used for maintaining the state of the processing
typedef struct abr_state_tag
{
    uint32_t es_bad_frame;      // bad frame count
    uint32_t es_inst_frames;    // instable frame count
    uint32_t es_stable_frames;  // calibration stable frame count
    uint32_t es_cqbuf;
    uint32_t es_frame_counter;
    int32_t abr_rms;
    int32_t es_peak_max;
    uint32_t es_best_class;
    int32_t frameBuffer[256];
    uint32_t frameBufferIdx;
    int32_t temp[256];
    int32_t es_qbuffer[ABR_NUMBER_QBUFS][ABR_NUMBER_SAMPLES];
    uint32_t es_qcounter[ABR_NUMBER_QBUFS];
    uint32_t es_offset[ABR_NUMBER_QBUFS];
    uint32_t es_peaks[ABR_NUMBER_QBUFS];
    uint32_t binom_skip_stat;
    uint32_t skip_frames;
    buffer notch_buf;
    int32_t abr_dc_max;
    notch_state_t vnotch_state;
    int32_t vertex_flag;
    int32_t nape_flag;
    int32_t vertex_common_imp;
    int32_t nape_common_imp;
    int32_t imp_balance;
    int32_t impedance_ok;
    int32_t pause_flag;
    int32_t imp_sum[48];

    uint32_t time_progress;

    uint32_t es_pe_frames;   // calibration probe error count
    uint32_t es_frame_wait;  // calibration delay to gain change
    uint32_t imp_counter;	// max number of frames
    size_t es_start;  // Start position of test
    size_t es_stop;   // End position of test
    uint32_t es_sqrtfact;
    int32_t prog_flag;
    uint32_t es_frame_counter_imp;
    uint32_t abr_imp_switch;
    uint32_t abr_init_imp_flag;	// When non-zero, initial impedance has been completed.
    uint32_t imp_stop_flag;
    uint32_t abr_stop_flag;
    int32_t abr_result;  // bug 3773 fix by saving result for reporting progress to screener service.

    // crc must be last in the structure
    uint32_t crc_initialized;
} abr_state_t;




#define ABR_STATE_SIZE  (sizeof(abr_state_t) - sizeof(uint32_t))

// Process return codes
// Negative values are used for Error conditions
typedef enum abr_return_tag
{
    ABR_RETURN_SUCCESS_RUNNING = 0,
    ABR_RETURN_SUCCESS_FINISHED = 1,
    ABR_RETURN_SUCCESS_TIME_LIMIT = 2,
    ABR_RETURN_SUCCESS_CANNOT_CALB = 3,
    ABR_RETURN_SUCCESS_COMPLETE_CALB = 4,
    ABR_RETURN_SUCCESS_INCREASE_GAIN = 5,
    ABR_RETURN_SUCCESS_DECREASE_GAIN = 6,
    ABR_RETURN_SUCCESS_PROBE_ERROR = 7,
    ABR_RETURN_SUCCESS_STARTED = 8,
    ABR_RETURN_SUCCESS_IMP_IDLE = 9,
    ABR_RETURN_SUCCESS_IMP_SWITCH = 10,
    ABR_RETURN_SUCCESS_IMP_GO = 11,
    ABR_RETURN_SUCCESS_IMP_DONE = 12,
    ABR_RETURN_SUCCESS_IMP_TIMEOUT = 13,
    ABR_RETURN_FAIL_INITIALIZED = -1,
    ABR_RETURN_FAIL_NULLSTATUS = -2,
    ABR_RETURN_FAIL_NULLSTATE = -3,
    ABR_RETURN_FAIL_NULLDATA = -4,
    ABR_RETURN_FAIL_CRC = -6,
    ABR_RETURN_FAIL_INVALID_SAMPLERATES = -7,
    ABR_RETURN_FAIL_INVALID_ES_START = -8,
    ABR_RETURN_FAIL_INVALID_ES_STOP = -9,
    ABR_RETURN_FAIL_INVALID_ES_SQRTFACT = -10,
    ABR_RETURN_FAIL_INVALID_QB_INDEX = -11,
    ABR_RETURN_FAIL_INVALID_FRAME_COUNT = -12

} abr_return_e;


// Sytem gain state reported to calibration
// so a probe error can be detected
typedef enum abr_gainstate_tag
{
        ABR_GAINSTATE_IN_RANGE = 0,
        ABR_GAINSTATE_AT_MAX = 1,
        ABR_GAINSTATE_AT_MIN = -1
} abr_gainstate_e;

typedef enum {
    ABR_PROG_UPDATE_LEAK = 0,
    ABR_PROG_UPDATE_NOISE = 1,
    ABR_PROG_UPDATE_CLIPPING = 2,
    ABR_PROG_UPDATE_OVERLOAD = 3,
    ABR_PROG_CHECK_PROBE = 4,
    ABR_PROG_UPDATE_NONE = 0xFF
}ABR_PROG_UPDATE_T;
/*
typedef enum {
    ABR_STATE_IDLE,
    ABR_STATE_IMP_INIT,
    ABR_STATE_IMP,
    ABR_STATE_IMP_WAIT,
    ABR_STATE_CAL_INIT,
    ABR_STATE_CAL,
    ABR_STATE_TEST_INIT,
    ABR_STATE_TEST
}ABR_STATE_T;*/

typedef enum {
	ABR_RESULT_PASS          = 0,
	ABR_RESULT_FAIL          = 1,
	ABR_RESULT_ABORT         = 2,
	ABR_RESULT_CALFAIL       = 3,
	ABR_RESULT_PROBE         = 4,
	ABR_RESULT_SYSTEM        = 5,
	ABR_RESULT_NONE          = 0xFF
}ABR_RESULT_T;


//

typedef enum {
    ABR_STATE_IDLE,
    ABR_STATE_PRETEST_IMPEDANCE_CHK_INIT,
    ABR_STATE_PRETEST_IMPEDANCE_CHK,
    ABR_STATE_PRETEST_IMPEDANCE_CHK_TEST,
    ABR_STATE_CALIBRATION_INIT,
    ABR_STATE_CALIBRATION,
    ABR_STATE_TESTING,
    ABR_STATE_PAUSED
}ABR_STATE_T;



typedef enum {
    ABR_TXT_STIMULUS_HIGH   = 0,
    ABR_TXT_STIMULUS_LOW    = 1,
    ABR_TXT_TOO_MUCH_NOISE  = 2,
    ABR_TXT_INTERFERENCE    = 3
}ABR_TEST_TXT_T;



/*! \brief teoae_calibration
 *
 *  This function calibartes the input level.
 *
 * \param[in/out] state Internal state structure
 * \param[out] status Exposed status structure
 * \param[in] data sampled data in a R3111 Frame
 * \param[in] gainstate The state of the system gain control
 * \return teoae_return_e
 *
 * \note The return code will provide direction to
 * change the incoming signal level.
 * Please review source code to see the return codes that must be handled.
 *
 */
abr_return_e abr_calibration(
    abr_state_t *state,
    abr_status_t *status);

abr_return_e abr_imp(
	abr_state_t *state,
	abr_status_t *status,
    int32_t data[]);

abr_return_e abr_imp_test(
	abr_state_t *state,
	abr_status_t *status,
    int32_t data[]);

uint32_t es_rms_lp(int32_t buff[], int32_t start, int32_t end);
void abr_cal_set_volume(abr_state_t *state);

abr_return_e abr_start(
    size_t es_start,
    size_t es_stop,
    uint32_t es_sqrtfact,
    abr_state_t *state);

abr_return_e abr_analyze(
	 int32_t *abr_result,
	 abr_state_t *state,
     int32_t data[256],
     size_t num_samples);

int32_t calc_impedance(abr_state_t *state, int32_t *data);

#endif //NATUS_ABR_ES3_H

