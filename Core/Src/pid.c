//
// Created by user on 2026/10/3.
//

#include "pid.h"
#include <math.h>
#include <stddef.h>

PID_Status_t PID_Init(PID_t *pid, float kp, float ki, float kd, float max, float min, float dt, float i_max, float e_stable)
{
    if (pid == NULL)
    {
        return PID_ERR_NULL;
    }
    if ( dt <=0.0f )
    {
        return PID_ERR_DT ;
    }

    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->max = max;
    pid->min = min;
    pid->dt = dt;
    pid->e_stable = e_stable;
    pid->i_max = i_max;     //限制e_sum,
    pid->e_sum = 0 ;
    pid->error_last = 0 ;
    pid->error_i = 0 ;      //i使用的e, 死区不清零
    pid->error_pd = 0 ;     //pd使用的e,死区时清零

    return PID_OK;
}

float PID_Calculate(PID_t *pid, float target, float current)
{
    if (pid == NULL) { return 0.0f ; }

    float output ;
    pid->error_pd = target - current ;
    pid->error_i = pid->error_pd ;
    if ( fabsf( pid->error_pd ) < pid->e_stable ) { pid->error_pd = 0; }

    pid->e_sum += pid->error_i ;
    if (pid->e_sum >  pid->i_max) pid->e_sum =  pid->i_max;
    if (pid->e_sum < -pid->i_max) pid->e_sum = -pid->i_max;
    float i_term = pid->ki * pid->e_sum * pid->dt;

    output = pid->kp * pid->error_pd + i_term + pid->kd * (pid->error_pd - pid->error_last)/pid->dt ;

    pid->error_last = pid->error_pd ;

    if ( output < pid->min ) { output = pid->min ; }
    if ( output > pid->max ) { output = pid->max ; }
    return output ;
}