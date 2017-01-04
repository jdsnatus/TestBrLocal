/*!
 * \file abr.c
 * \author  Dan
 * \date 2012-04-20
 * \version 1.0
 *
 * \copyright Copyright (c) 2012 Natus Medical Incorporated.
 * All rights reserved.
 *
 * This software is the confidential and proprietary information of Natus
 * Medical Incorporated. ("Confidential Information").
 */
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "sharedMem.h"
#include "es3crc16.h"
#include "es3dsp.h"
#include "abr_es3.h"
#include "control.h"
#include "gpio.h"
#include "globals.h"
#include "buffer.h"
#include "abr_template_coef.h"

// Randomly selected number, lower 16 bits reserved for CRC
#define CRC_INITIALIZED (0x89760000)
#define CRC_INITIAL (0xFFFF)

extern int32_t mic_buff[1920];
// 08-24-2012 JDS bug 3773 use result value from check_result method.
abr_return_e abr_analyze(
	 int32_t *abr_result,
	 abr_state_t *state,
     int32_t data[256],
     size_t num_samples)
 {

	abr_return_e abr_return = ABR_RETURN_SUCCESS_RUNNING;

    int32_t sum =0;
    uint32_t temp =0;
    uint32_t binom_stat1 = 0;
    uint32_t binom_stat = 0;
    uint32_t sumsqrs = 0;
    int32_t old_rms = 0;
    size_t qb = 0;
    size_t i,j = 0;
    buffer *notch_ptr = &state->notch_buf;
    buffer *vnotch_ptr = &state->vnotch_state.vnotch_buf;
    buffer *vnotch_dptr = &state->vnotch_state.vnotch_dbuf;
    *abr_result = ABR_RESULT_NONE;  // not known result yet


    /*Relocate per the controller's continuous time filtering
    es3_notch(
        data,
        state->temp,
        notch_ptr,
        num_samples,
        (uint32_t)170);  //170 is the notch filter delay

	es3_adj_notch(
        int32_t in[],
        int32_t out[],
        int32_t abr_dc_filter[],
        num_samples,
        uint32_t fs,
        size_t *notch_clip_flag)

    es3_hi_pass(
        int32_t in[],
        int32_t out[],
        int32_t abr_dc_filter[],
        state->abr_dc_max,
        num_samples)
	*/

    /*TODO:
    es3_acoustic_noise_check(mic_buff,
    						 num_samples,
							 threshold);
	*/

    es3_convolution(
                    abr_template,
                    data,
                    num_samples,
                    state->temp,
                    256);

    es3_binomial_stats(
                        state->temp,
                        &binom_stat,
                        &binom_stat1,
                        &sumsqrs);

    old_rms = state->abr_rms;

    state->es_best_class = sumsqrs;
    //printf("sumsq: %x %u\n",sumsqrs,sumsqrs);
    state->abr_rms = es3_sqrt_assy(sumsqrs);

    //apply a time constant to the EEG level
    state->abr_rms = (old_rms * ABR_RMS_TC + state->abr_rms * ABR_TC + 16384) >> 15;

    state->skip_frames = frameAccpetance(
                                            binom_stat,
                                            binom_stat1,
                                            &state->binom_skip_stat,  //needs investigation
                                            state->abr_rms,
                                            old_rms, //old rms value
                                            state->abr_dc_max); //dc_max from filter


    if(state->skip_frames <= 0)
    {
        //update qbuffers
        state->es_cqbuf = update_qbuffers(
                                    es_tres,
                                    state->abr_rms,
                                    state->es_qbuffer,
                                    state->temp,
                                    state->es_offset,
                                    state->es_qcounter);


        //criteria calculation
        criteria_calc(
            state->es_qcounter,
            state->es_qbuffer,
            state->es_peaks,
            state->es_cqbuf);

        //check result
        *abr_result = check_result(&state->es_peak_max,
        	state->es_frame_counter,
        	state->es_peaks);

    }


    return abr_return;
 }

/// \brief teoae_calibration
abr_return_e abr_imp(
	abr_state_t *state,
	abr_status_t *status,
    int32_t data[])
{
    abr_return_e abr_return = ABR_RETURN_SUCCESS_RUNNING;
    uint16_t crc = CRC_INITIAL;
    int32_t i;
    int32_t dc_offset = 0;
    int32_t impedance = 0;

    int32_t es_per1_max = 0;
    int32_t es_ring_max = 0;

    uint32_t es_sb_max = 0;

    if (status == NULL)
    {
        //ERROR("NULL status pointer");
    	abr_return = ABR_RETURN_FAIL_NULLSTATUS;
        goto _abr_imp;
    }

    if (state == NULL)
    {
        //ERROR("NULL state pointer");
    	abr_return = ABR_RETURN_FAIL_NULLSTATE;
        goto _abr_imp;
    }

    if (data == NULL)
    {
        //ERROR("NULL data pointer");
    	abr_return = ABR_RETURN_FAIL_NULLDATA;
        goto _abr_imp;
    }

    //if ((state->crc_initialized & 0xFFFF0000) != CRC_INITIALIZED)
    //{
        //ERROR("Not initialized 0x%x", state->crc_initialized);
    //	abr_return = ABR_RETURN_FAIL_INITIALIZED;
    //    goto _abr_imp;
    //}

    //crc = CRC_INITIAL;
    //es3_crc16(state, ABR_STATE_SIZE, &crc);
    //if ((state->crc_initialized & 0x0000FFFF) != (uint32_t)crc)
    //{
    //    //ERROR("CRC mismatch");
    //	abr_return = ABR_RETURN_FAIL_CRC;
    //    goto _abr_imp;
    //}

    //state->es_frame_counter++;
    state->es_frame_counter_imp++;

    if(state->es_frame_counter_imp > 25)
    {
    	state->es_frame_counter_imp = 0;
    	if(state->imp_counter++ < IMP_MAX_FRAMES)
    		abr_return = ABR_RETURN_SUCCESS_IMP_GO;
    	else
    		abr_return = ABR_RETURN_SUCCESS_IMP_TIMEOUT;
    }
    else if(state->es_frame_counter_imp == 4)
    {
    	impedance = calc_impedance(state, data);

    	if(state->vertex_flag)
    		state->vertex_common_imp = impedance;
    	else
    		state->nape_common_imp = impedance;

        abr_return = ABR_RETURN_SUCCESS_IMP_IDLE;
    }
    else if(state->es_frame_counter_imp == 2)
	{
    	impedance = calc_impedance(state, data);

    	if(state->vertex_flag)
    		state->vertex_common_imp = impedance;
    	else
    		state->nape_common_imp = impedance;

		abr_return = ABR_RETURN_SUCCESS_IMP_SWITCH;
	}


#if 0
    // Update the state CRC
	crc = CRC_INITIAL;
	es3_crc16(state, STATE_SIZE, &crc);
	state->crc_initialized = CRC_INITIALIZED | (uint32_t)crc;
#endif

_abr_imp:

    return abr_return;
}

abr_return_e abr_imp_test(
	abr_state_t *state,
	abr_status_t *status,
    int32_t data[])
{
    abr_return_e abr_return = ABR_RETURN_SUCCESS_RUNNING;
    uint16_t crc = CRC_INITIAL;
    int32_t i;
    int32_t dc_offset = 0;
    int32_t impedance = 0;

    int32_t es_per1_max = 0;
    int32_t es_ring_max = 0;

    uint32_t es_sb_max = 0;

    if (status == NULL)
    {
        //ERROR("NULL status pointer");
    	abr_return = ABR_RETURN_FAIL_NULLSTATUS;
        goto _abr_imp;
    }

    if (state == NULL)
    {
        //ERROR("NULL state pointer");
    	abr_return = ABR_RETURN_FAIL_NULLSTATE;
        goto _abr_imp;
    }

    if (data == NULL)
    {
        //ERROR("NULL data pointer");
    	abr_return = ABR_RETURN_FAIL_NULLDATA;
        goto _abr_imp;
    }

    //state->es_frame_counter++;
    state->es_frame_counter_imp++;

    if(state->es_frame_counter_imp == 4)
    {
    	impedance = calc_impedance(state, data);

    	if(state->vertex_flag)
    		state->vertex_common_imp = impedance;
    	else
    		state->nape_common_imp = impedance;

    	state->es_frame_counter_imp = 0;

        abr_return = ABR_RETURN_SUCCESS_IMP_DONE;
    }
    else if(state->es_frame_counter_imp == 2)
	{
    	impedance = calc_impedance(state, data);

    	if(state->vertex_flag)
    		state->vertex_common_imp = impedance;
    	else
    		state->nape_common_imp = impedance;

		abr_return = ABR_RETURN_SUCCESS_IMP_SWITCH;
	}

_abr_imp:

    return abr_return;
}

/// \brief teoae_calibration
abr_return_e abr_calibration(
	abr_state_t *state,
	abr_status_t *status)
{
    abr_return_e abr_return = ABR_RETURN_SUCCESS_RUNNING;
    uint16_t crc = CRC_INITIAL;
    int32_t es_per1_max = 0;
    int32_t es_ring_max = 0;

    uint32_t es_sb_max = 0;

    if (status == NULL)
    {
        //ERROR("NULL status pointer");
    	abr_return = ABR_RETURN_FAIL_NULLSTATUS;
        goto _abr_calibration;
    }

    if (state == NULL)
    {
        //ERROR("NULL state pointer");
    	abr_return = ABR_RETURN_FAIL_NULLSTATE;
        goto _abr_calibration;
    }

#if 0
    if ((state->crc_initialized & 0xFFFF0000) != CRC_INITIALIZED)
    {
        //ERROR("Not initialized 0x%x", state->crc_initialized);
    	abr_return = ABR_RETURN_FAIL_INITIALIZED;
        goto _abr_calibration;
    }

    crc = CRC_INITIAL;
    es3_crc16(state, ABR_STATE_SIZE, &crc);
    if ((state->crc_initialized & 0x0000FFFF) != (uint32_t)crc)
    {
        //ERROR("CRC mismatch");
    	abr_return = ABR_RETURN_FAIL_CRC;
        goto _abr_calibration;
    }
#endif

    //state->es_frame_counter++;

    // calc the rms of the feedback signal
    es_sb_max = es_rms_lp(state->frameBuffer, ABR_STIM_CONTROL_START, ABR_STIM_CONTROL_STOP);

    status->Percent_Noise_Indicator = es_sb_max*100/2/ABR_STIM_LEVEL;	// Calculate calibration level used for display (0-100)
    if(status->Percent_Noise_Indicator > 100)
    	status->Percent_Noise_Indicator = 100;


    if (es_sb_max < ABR_CAL_LO)
	{
    	abr_cal_set_volume(state);
    	abr_return = ABR_RETURN_SUCCESS_INCREASE_GAIN;
	}
    else if (es_sb_max > ABR_CAL_HI)
    {
    	abr_cal_set_volume(state);
    	abr_return = ABR_RETURN_SUCCESS_DECREASE_GAIN;
    }
    else
    {
    	state->es_stable_frames++;
    	if(state->es_stable_frames >= ABR_STABLE_UNTIL_CONT)
    	{
    		abr_return = ABR_RETURN_SUCCESS_COMPLETE_CALB;
    		goto _abr_calibration;
    	}
    }

    if(state->es_frame_counter >= ABR_CAL_MAXFRAMES)
    	abr_return = ABR_RETURN_SUCCESS_CANNOT_CALB;		// SUSPENDED

#if 0
    // Update the state CRC
	crc = CRC_INITIAL;
	es3_crc16(state, STATE_SIZE, &crc);
	state->crc_initialized = CRC_INITIALIZED | (uint32_t)crc;
#endif

_abr_calibration:

    return abr_return;
}

// buff - buffer to calculate rms value for.
uint32_t es_rms_lp(int32_t buff[], int32_t start, int32_t end)
{
	int32_t i=0;			// loop counter
    int32_t temp = 0;
	uint32_t es_sb_max = 0;
	int32_t sample = 0;
	int32_t cnt = 0;

	cnt = start;
	sample = buff[cnt];
	while(cnt < end)
	{
		temp += (sample>>2) * (sample>>2);
		sample = sample>>1 + buff[++cnt];
	}

	es_sb_max = (int32_t)floor(sqrt((float)temp));
	//es3_sqrt((uint32_t *)&temp, &es_sb_max, 1);		// fixed point sqrt

	return es_sb_max;
}

void abr_cal_set_volume(abr_state_t *state)
{
	state->es_bad_frame++;

	if(state->es_bad_frame >= ABR_CAL_WHEN_SHOW_MSG)
		state->prog_flag = ABR_PROG_CHECK_PROBE;

	state->es_stable_frames = 0;	// reset stable frames.
	// Display canal bar
}


/// \brief abr_start
abr_return_e abr_start(
    size_t es_start,
    size_t es_stop,
    uint32_t es_sqrtfact,
    abr_state_t *state)
 {


     abr_return_e abr_return = ABR_RETURN_SUCCESS_STARTED;
     uint16_t crc = CRC_INITIAL;
     size_t qb = 0; // buffer index

#if 0
     // assert test -> this design decimates the input signal
     if (INPUT_SR < OUTPUT_SR)
     {
    	 abr_return = ABR_RETURN_FAIL_INVALID_SAMPLERATES;
         // ERROR("ASSERT INPUT_SR %d < OUTPUT_SR %d", INPUT_SR, OUTPUT_SR);
         goto _abr_start;
     }

     // assert test -> this design requires an integer multiple SR
     if ((INPUT_SR % OUTPUT_SR) != 0)
     {
    	 abr_return = ABR_RETURN_FAIL_INVALID_SAMPLERATES;
         // ERROR("ASSERT INPUT_SR %d % OUTPUT_SR %d", INPUT_SR, OUTPUT_SR);
         goto _abr_start;
     }
#endif

     // Clear the state variable
     memset(state, 0, sizeof(abr_state_t));
     state->abr_rms = 1;

#if 0
     // Set ES_START
     if (es_start >= ABR_NUMBER_SAMPLES)
     {
    	 abr_return = ABR_RETURN_FAIL_INVALID_ES_START;
         // ERROR("ASSERT es_start %d >= %d ", es_start, TEOAE_NUMBER_SAMPLES);
         goto _abr_start;
     }
     state->es_start = es_start;

     // Set ES_STOP
     if (es_stop > ABR_NUMBER_SAMPLES || es_start >= es_stop)
     {
    	 abr_return = ABR_RETURN_FAIL_INVALID_ES_STOP;
         // ERROR("ASSERT es_stop %d >= %d ", es_stop, TEOAE_NUMBER_SAMPLES);
         goto _abr_start;
     }
     state->es_stop = es_stop;

     // Set ES_SQRTFACT
     if (es_sqrtfact >= 100)
     {
    	 abr_return = ABR_RETURN_FAIL_INVALID_ES_SQRTFACT;
         // ERROR("ASSERT es_sqrtfact %d >= %d ", es_sqrtfact, 100);
         goto _abr_start;
     }
     state->es_sqrtfact = es_sqrtfact;

#endif

#if 0
     // initialize the peak buffer
     for (qb = 0; qb < ABR_NUMBER_QBUFFERS; qb++)
     {
        es3_docrit(
            &state->Q_es_qbuffer[qb][0],
            state->Q_sigLimit[qb],
            &state->Q_es_peaks[qb],
            &state->peaks[qb][0],
            (size_t)TEOAE_NUMBER_SAMPLES,
            (size_t)TEOAE_NUMBER_SAMPLES);
     }

     //Initialization for filtering buffers needs relocated
            initBuffer(notch_ptr);
            initBuffer(vnotch_ptr);
            initBuffer(vnotch_dptr);
            for(i=0;i<BUFF_SIZE;i++)
            {
              write(notch_ptr,0);
              write(vnotch_ptr,0);
              write(vnotch_dptr,0);
            }
            state->abr_rms = 1;
#endif


     // Compute CRC and Initialize
     es3_crc16(state, ABR_STATE_SIZE, &crc);
     state->crc_initialized = CRC_INITIALIZED | (uint32_t)crc;

_abr_start:
     return abr_return;
 }

//  calc_impedance()
// Modified
//   08-01-2012  JDS  Bug 3414  check for max value and force to 99k
//
int32_t val1 = 16;
int32_t val2 = 76;
int32_t val3 = 7;
int32_t calc_impedance(abr_state_t *state, int32_t *data)
{
	int32_t dc_offset = 0;
	uint64_t rms = 0;
	int32_t i;
	int32_t result;


	if(*(int32_t *)(BBRq_ADDR+0x20) != 0)
	{
		//val1 = *(int32_t *)0x8effb020;
		val1 = *(int32_t *)(BBRq_ADDR+0x20);
		*(int32_t *)(BBRq_ADDR+0x20) = 0;
	}
	if(*(int32_t *)(BBRq_ADDR+0x24) != 0)
	{
		val2 = *(int32_t *)(BBRq_ADDR+0x24);
		*(int32_t *)(BBRq_ADDR+0x24) = 0;
	}
	if(*(int32_t *)(BBRq_ADDR+0x28) != 0)
	{
		val3 = *(int32_t *)(BBRq_ADDR+0x28);
		*(int32_t *)(BBRq_ADDR+0x28) = 0;
	}

	for(i=0;i<ABR_IMP_CYCLE_LEN;i++)
		state->imp_sum[i] = 0;

	for(i=0;i<ABR_IMP_FRAMELEN;i++)
		state->imp_sum[i%ABR_IMP_CYCLE_LEN] += data[i]>>21;

	//sprintf(print_buf, "0x%08X ** 0x%08X\n", data[0]>>21, data[1]>>21); LogStuff(print_buf, strlen(print_buf));
	//sprintf(print_buf, "%d ** %d\n", data[0]>>21, data[1]>>21); LogStuff(print_buf, strlen(print_buf));

	for(i=0;i<ABR_IMP_CYCLE_LEN;i++)
		dc_offset += state->imp_sum[i];
	dc_offset /= ABR_IMP_CYCLE_LEN;
	//sprintf(print_buf, "dc_offset - %d\n", dc_offset); LogStuff(print_buf, strlen(print_buf));

	for(i=0;i<ABR_IMP_CYCLE_LEN;i++)
	{
		state->imp_sum[i] -= dc_offset;
		rms += state->imp_sum[i]*state->imp_sum[i];
	}
	rms = rms>>val3;
	//rms = rms>>2;

	rms = (uint32_t)sqrt((double)rms);

	if(rms < val1)
		rms = val1;
	//result = (rms - 43)*0.46;
	result = (rms - val1)*(val2/100.0);  // note divide by 100 will get multiplied by 100 from calling routine.

	if (result>ABR_IMP_MAX_IMP) result = ABR_IMP_MAX_IMP;  // check for overflow at max val of 99kohm

	return result;
}
