#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H
#include "pid.h"
#include "MahonyAHRS.h"

typedef struct {
    float stand_kp, stand_kd;
    float speed_kp, speed_ki;
    float turn_kp, turn_kd;
} BalanceParam_t;

typedef struct {
    PID_t stand_pid;
    PID_t speed_pid;
    PID_t turn_pid;

    EulerAngles_t status;

    BalanceParam_t param;

    float speed_now_left , speed_now_right , speed_now;
    float speed_set , turn_set;
    float gyro_pitch_v;
    float left_duty , right_duty;
} Balance_t;

void Balance_Init(Balance_t *balance , BalanceParam_t *param);

void Balance_UpdateFast(Balance_t *balance );

void Balance_UpdateSlow(Balance_t *balance );

#endif