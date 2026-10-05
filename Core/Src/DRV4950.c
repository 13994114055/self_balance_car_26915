//
// Created by user on 2026/10/5.
//

#include "DRV4950.h"

HAL_StatusTypeDef Motor_StatusSet(Motor_t *motor , WorkMode_t work_mode) {
    motor->work_mode = work_mode;
    switch (motor->work_mode) {
        case Motor_Stop  :
            TIM_CCxChannelCmd(motor->htim_pwr->Instance , motor->channel[0],DISABLE);
            TIM_CCxChannelCmd(motor->htim_pwr->Instance , motor->channel[1], DISABLE);
            break;
        case Motor_Normal:
            TIM_CCxChannelCmd(motor->htim_pwr->Instance, motor->channel[0], ENABLE);
            TIM_CCxChannelCmd(motor->htim_pwr->Instance, motor->channel[1], ENABLE);
            break;
        default: return HAL_ERROR;
    }
    return HAL_OK;
}

HAL_StatusTypeDef Motor_Init(Motor_t *motor , TIM_HandleTypeDef *htim , uint32_t channelIN1 , uint32_t channelIN2 ,WorkMode_t work_mode) {
    motor->htim_pwr = htim;
    motor->channel[0] = channelIN1;
    motor->channel[1] = channelIN2;
    motor->pwr_min = 0.05f ;
    motor->pwr_max = 0.95f ;

    HAL_TIM_PWM_Start(motor->htim_pwr, motor->channel[0]);
    HAL_TIM_PWM_Start(motor->htim_pwr, motor->channel[1]);

    Motor_StatusSet(motor , work_mode) ;


    return HAL_OK ;
}

void Motor_DutySet(Motor_t *motor, float duty) {
    int sign = (duty>=0) - (duty<0);
    float uduty = sign * duty;
    uint16_t uoutput ;
    uoutput = uduty * __HAL_TIM_GET_AUTORELOAD(motor->htim_pwr);


    if (sign >= 0) {
        if (uduty < motor->pwr_min) {
            uoutput = 0.0f ;
            __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[1] ,uoutput) ;
        }
        else if (uduty > motor->pwr_max) {
            uoutput = motor->pwr_max * __HAL_TIM_GET_AUTORELOAD(motor->htim_pwr) ;
            __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[1] ,uoutput);
        }
        else {
            __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[1] ,uoutput);
        }

        __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[0] ,0.0f);
    }
    else {
        if (uduty < motor->pwr_min) {
            uoutput = 0.0f ;
            __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[0] ,uoutput) ;
        }
        else if (uduty > motor->pwr_max) {
            uoutput = motor->pwr_max * __HAL_TIM_GET_AUTORELOAD(motor->htim_pwr) ;
            __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[0] ,uoutput);
        }
        else {
            __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[0] ,uoutput);
        }

        __HAL_TIM_SET_COMPARE(motor->htim_pwr , motor->channel[1] ,0.0f);

    }
}


