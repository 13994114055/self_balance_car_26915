#ifndef __DRV4950_H
#define __DRV4950_H

/**
 * @file   DRV4950.h
 * @brief  DRV4950 双 H 桥电机驱动接口
 */

#include "tim.h"

/**
 * @brief 工作模式：PWM 输出是否使能
 */
typedef enum {
    Motor_Stop = 0,     /* 停止：禁用 PWM 输出（电机自由滑行） */
    Motor_Normal = 1    /* 正常运行：启用 PWM 输出 */
} WorkMode_t;

/**
 * @brief 刹车模式【已废案】
 * @note  平衡车用滑行停即可；如需急停，另做 Motor_Brake()（短路制动）
 */
typedef enum {
    FastStop = 0,   /* 快速停：一通道 PWM、另一通道关断（驱动 ↔ 滑行） */
    LowStop = 1     /* 低速停：一通道常开、另一通道 PWM（驱动 ↔ 刹车） */
} BrakeMode_t;


/**
 * @brief 电机实例句柄：硬件绑定 + 配置
 */
typedef struct {
    TIM_HandleTypeDef *htim_pwr ;       /* 驱动本电机的 PWM 定时器句柄 */
    uint32_t channel[2] ;               /* H 桥两个输入通道 [IN1, IN2] */
    float pwr_max ;                     /* 输出上限（归一化，如 0.97） */
    float pwr_min ;                     /* 输出下限（归一化，如 0.03） */
    WorkMode_t work_mode ;              /* 工作模式：Stop / Normal */
}Motor_t;


/**
 * @brief  初始化电机实例：绑定定时器/通道，设定初始模式
 * @param  motor      : 指向电机实例的指针
 * @param  htim       : PWM 定时器句柄（如 &htim2）
 * @param  channelIN1 : H 桥 IN1 通道（如 TIM_CHANNEL_1）
 * @param  channelIN2 : H 桥 IN2 通道（如 TIM_CHANNEL_2）
 * @param  work_mode  : 初始工作模式（Motor_Stop=禁用输出 / Motor_Normal=启用）
 * @return HAL_OK 成功；HAL_ERROR 失败
 */
HAL_StatusTypeDef Motor_Init(Motor_t *motor , TIM_HandleTypeDef *htim , uint32_t channelIN1 , uint32_t channelIN2 ,WorkMode_t work_mode) ;

/**
 * @brief  设置电机的工作模式（启停）
 * @param  motor     : 指向电机实例的指针
 * @param  work_mode : 工作模式（Motor_Stop=禁用PWM输出 / Motor_Normal=启用）
 * @return HAL_OK 成功；HAL_ERROR 失败
 */
HAL_StatusTypeDef Motor_StatusSet(Motor_t *motor , WorkMode_t work_mode ) ;

/**
 * @brief  设置电机占空比
 * @param  motor : 指向电机实例的指针
 * @param  duty  : 占空比，范围 -1.0 ~ +1.0（正负=方向，绝对值=力度）
 * @note   输出会夹在 [pwr_min, pwr_max]；duty=0 时电机滑行
 */
void Motor_DutySet(Motor_t *motor, float duty);

#endif
