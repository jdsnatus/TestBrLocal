#ifndef ABR_H
#define ABR_H
/***********************************************************************

    (C) Copyright 2016. Natus Medical Inc. All Rights Reserved.

Name
    abr.h

Description
   2017-05-01 JDS include file for abr module.

Notes

----------------------------------------------------------------------*/

#include <stdint.h>
#include "globals.h"

/*
typedef enum {
    ABR_STATE_IDLE,
    ABR_STATE_PRETEST_IMPEDANCE_CHK,
    ABR_STATE_CALIBRATION,
    ABR_STATE_TESTING,
    ABR_STATE_PAUSED
}ABR_STATE_T;

typedef enum {
    ABR_RESULT_PASS          = 0,
    ABR_RESULT_FAIL          = 1,
    ABR_RESULT_ABORT         = 2,
    ABR_RESULT_CALFAIL       = 3,
    ABR_RESULT_PROBE         = 4,
    ABR_RESULT_SYSTEM        = 5,
    ABR_RESULT_NONE          = 0xFF
}ABR_RESULT_T;



typedef enum {
    ABR_TXT_STIMULUS_HIGH   = 0,
    ABR_TXT_STIMULUS_LOW    = 1,
    ABR_TXT_TOO_MUCH_NOISE  = 2,
    ABR_TXT_INTERFERENCE    = 3
}ABR_TEST_TXT_T;






*/

// api
CTRL_RETURN_T abr_process(DATA_IN_T *data_in);
void abr_send_cal_prog(abr_state_t *score, abr_status_t *status);
void abr_send_test_prog(abr_state_t *score);
void abr_send_imp_prog(abr_state_t *score );
void abr_send_test_done(abr_state_t *score);

#endif // ABR_H
