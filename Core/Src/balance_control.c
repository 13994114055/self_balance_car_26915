//
// Created by user on 2026/10/5.
//

#include "balance_control.h"

#include <math.h>
#include <stdint.h>


void Balance_Init(Balance_t *balance , BalanceParam_t *param) {
    balance->param = *param;
    PID_Init(&balance->stand_pid , balance->param.stand_kp , 0 , balance->param.stand_kd , 0.5f,-0.5f,0.001f, 0 , 0);
    PID_Init(&balance->speed_pid , balance->param.speed_kp , balance->param.speed_ki , 0 , 10 , -10 , 0.02f , 5 , 0.02);
    PID_Init(&balance->turn_pid , balance->param.turn_kp , 0 , balance->param.turn_kd , 0.5 , -0.5 , 0.02f , 0 , 1);
    balance->status.yaw =  0.0f ;
    balance->status.pitch =  0.0f ;
    balance->status.roll =  0.0f ;
    balance->speed_now = 0.0f ;
    balance->speed_now_left  = 0.0f ;
    balance->speed_now_right = 0.0f ;
    balance->speed_set = 0.0f ;
    balance->turn_set = 0.0f ;
    balance->gyro_pitch_v = 0.0f ;
    balance->left_duty = 0.0f ;
    balance->right_duty = 0.0f ;
    balance->pitch_target = 0.0f ;
    balance->turn_out = 0.0f ;
    balance->balance_out = 0.0f ;
}

void Balance_UpdateSlow(Balance_t *balance ) {
    balance->speed_now = ( balance->speed_now_left + balance->speed_now_right ) * 0.5f ;
    balance->pitch_target = PID_Calculate(&balance->speed_pid , balance->speed_set , balance->speed_now) ;
    balance->turn_out = PID_Calculate(&balance->turn_pid , balance->turn_set , balance->status.yaw) ;
}

void Balance_UpdateFast(Balance_t *balance ) {
    float error = balance->pitch_target - balance->status.pitch ;
      //int8_t sign = (error > 0) - (error < 0);
    balance->balance_out = -1 * (balance->stand_pid.kp * error - balance->stand_pid.kd * balance->gyro_pitch_v );
    balance->left_duty = balance->balance_out + balance->turn_out ;
    balance->right_duty = balance->balance_out - balance->turn_out ;

    if (balance->left_duty  >  1.0f) balance->left_duty  =  1.0f;
    if (balance->left_duty  < -1.0f) balance->left_duty  = -1.0f;
    if (balance->right_duty >  1.0f) balance->right_duty =  1.0f;
    if (balance->right_duty < -1.0f) balance->right_duty = -1.0f;
}
