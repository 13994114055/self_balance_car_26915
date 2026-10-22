#ifndef __BALANCE_CONTROL_H
#define __BALANCE_CONTROL_H
#include "pid.h"
#include "MahonyAHRS.h"

/**
 * @file   balance_control.h
 * @brief  三环串级控制：直立(PD) + 速度(PI) + 转向(PD) → 左右占空比
 * @note   纯逻辑模块，不依赖硬件驱动；由任务层填充输入、读取输出
 */

/**
 * @brief 三环增益配置（Init 时注入）
 */
typedef struct {
    float stand_kp, stand_kd;   /* 直立环：比例 / 微分（D 用原始陀螺） */
    float speed_kp, speed_ki;   /* 速度环：比例 / 积分 */
    float turn_kp, turn_kd;     /* 转向环：比例 / 微分 */
} BalanceParam_t;

/**
 * @brief 三环控制实例：状态(3个PID) + 配置 + 输入数据 + 中间量 + 输出
 */
typedef struct {
    PID_t stand_pid;            /* 直立环 PID（当前用手动 PD，保留占位） */
    PID_t speed_pid;            /* 速度环 PID（PI，50Hz） */
    PID_t turn_pid;             /* 转向环 PID（PD，50Hz） */

    EulerAngles_t status;       /* 当前姿态（任务层从 Mahony 填入，单位：度） */

    BalanceParam_t param;       /* 增益配置 */

    float speed_now_left , speed_now_right , speed_now; /* 左右轮实际速度(脉冲/周期) / 平均速度 */
    float speed_set , turn_set;                          /* 目标速度 / 目标转向 */
    float gyro_pitch_v;                                  /* pitch 轴角速度(dps)，直立环 D 项用 */
    float left_duty , right_duty;                        /* 输出：左右电机占空比(-1~1) */
    float pitch_target;                                  /* 中间量：速度环输出 = 期望倾角(度) */
    float turn_out;                                      /* 中间量：转向环输出(差速) */
    float balance_out;                                   /* 中间量：直立环输出(基础占空比) */
} Balance_t;

/**
 * @brief  初始化三环控制实例
 * @param  balance : 指向实例的指针
 * @param  param   : 增益配置（会被拷贝进实例）
 * @note   速度/转向环在此初始化 PID；直立环为手动 PD
 */
void Balance_Init(Balance_t *balance , BalanceParam_t *param);

/**
 * @brief  快环（1kHz）：直立环 + 左右合成 + 输出限幅
 * @param  balance : 指向实例的指针
 * @note   使用 UpdateSlow 算出的 pitch_target / turn_out
 */
void Balance_UpdateFast(Balance_t *balance );

/**
 * @brief  慢环（50Hz）：速度环 → pitch_target；转向环 → turn_out
 * @param  balance : 指向实例的指针
 */
void Balance_UpdateSlow(Balance_t *balance );

#endif
