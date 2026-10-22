#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f1xx_hal_tim.h"


typedef struct {
    TIM_HandleTypeDef*  htim_left;
    TIM_HandleTypeDef*  htim_right;
    uint16_t cnt_left_last ;
    uint16_t cnt_right_last ;
    uint16_t cnt_now_left ;
    uint16_t cnt_now_right ;
    float speed_left;
    float speed_right;
}Encoder_t ;


void Encoder_Init(Encoder_t* enc , TIM_HandleTypeDef* tim_left , TIM_HandleTypeDef* tim_right );

void Encoder_Update(Encoder_t* enc) ;

#endif