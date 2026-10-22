#ifndef __ENCODER_H
#define __ENCODER_H
#include "stm32f1xx_hal_tim.h"

/**
 * @file   encoder.h
 * @brief  编码器测速模块（TIM3/TIM4 硬件编码器模式）
 * @note   速度单位：脉冲/采样周期（不换算 m/s）
 */

/**
 * @brief 编码器实例：硬件绑定 + 上次计数 + 速度
 */
typedef struct {
    TIM_HandleTypeDef* htim_left;    /* 左轮编码器定时器（指向 &htim3） */
    TIM_HandleTypeDef* htim_right;   /* 右轮编码器定时器（指向 &htim4） */

    uint16_t cnt_left_last;          /* 左轮上次 CNT（0~65535） */
    uint16_t cnt_right_last;         /* 右轮上次 CNT */
    uint16_t cnt_now_left;           /* 左轮本次 CNT */
    uint16_t cnt_now_right;          /* 右轮本次 CNT */

    float speed_left;                /* 左轮速度（脉冲/周期，正=正转） */
    float speed_right;               /* 右轮速度（脉冲/周期） */
}Encoder_t ;

/**
 * @brief  初始化编码器实例
 * @param  enc       : 指向实例的指针
 * @param  tim_left  : 左轮编码器定时器（&htim3）
 * @param  tim_right : 右轮编码器定时器（&htim4）
 * @note   读取一次当前 CNT 作为初值，避免第一次差值异常
 */
void Encoder_Init(Encoder_t* enc , TIM_HandleTypeDef* tim_left , TIM_HandleTypeDef* tim_right );

/**
 * @brief  更新左右轮速度（周期调用，建议 10ms）
 * @param  enc : 指向实例的指针
 * @note   用 16 位有符号差值处理 CNT 回绕；单周期差值需在 ±32767 以内
 */
void Encoder_Update(Encoder_t* enc) ;

#endif
