//
// Created by user on 2026/10/22.
//

#include "encoder.h"


void Encoder_Init(Encoder_t* enc , TIM_HandleTypeDef* htim_left , TIM_HandleTypeDef* htim_right ){
    enc->htim_left  = htim_left;
    enc->htim_right = htim_right;
    enc->cnt_left_last  = enc->htim_left->Instance->CNT  ;
    enc->cnt_right_last = enc->htim_right->Instance->CNT ;
    enc->cnt_now_left = 0 ;
    enc->cnt_now_right = 0 ;
    enc->speed_left  = 0.0f;
    enc->speed_right = 0.0f;
}

void Encoder_Update(Encoder_t* enc) {
    enc->cnt_now_left = enc->htim_left->Instance->CNT ;
    enc->cnt_now_right = enc->htim_right->Instance->CNT ;
    int16_t diff_left = enc->cnt_now_left - enc->cnt_left_last;
    int16_t diff_right = enc->cnt_now_right - enc->cnt_right_last;
    enc->cnt_left_last = enc->cnt_now_left ;
    enc->cnt_right_last = enc->cnt_now_right ;
    enc->speed_left = diff_left ;
    enc->speed_right = diff_right ;
}
