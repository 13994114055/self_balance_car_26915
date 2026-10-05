#ifndef __MAHONY_AHRS_H
#define __MAHONY_AHRS_H

/**
 * @file   MahonyAHRS.h
 * @brief  Mahony 互补滤波姿态解算（纯算法，不依赖硬件）
 */

/**
 * @brief Mahony AHRS 实例句柄（只存状态与配置）
 */
typedef struct {
    float q[4];              /* 姿态四元数 [q0,q1,q2,q3]（内部状态） */
    float kp ;               /* 比例增益（加速度计修正强度），经验值 2.0 */
    float ki ;               /* 积分增益（陀螺零偏观测器），经验值 0.05 */
    float integralFB[3] ;    /* 积分反馈项（在线估计陀螺零偏） */

}MahonyAHRS_t ;

/**
 * @brief 欧拉角（输出，单位：度）
 */
typedef struct {
    float yaw ;              /* 偏航角 (度) */
    float pitch ;            /* 俯仰角 (度) */
    float roll ;             /* 滚转角 (度) */
}EulerAngles_t ;

/**
 * @brief  初始化 Mahony 滤波器实例
 * @param  maho : 指向实例的指针
 * @param  kp   : 比例增益（经验值 2.0）
 * @param  ki   : 积分增益（经验值 0.05）
 * @note   四元数初始化为 (1,0,0,0)，积分项清零
 */
void MahonyAHRS_Init(MahonyAHRS_t *maho , float kp , float ki ) ;

/**
 * @brief  用一组传感器数据更新一次姿态
 * @param  maho     : 指向实例的指针
 * @param  gyro[3]  : 陀螺仪角速度 [x,y,z]，单位 dps
 * @param  accel[3] : 加速度计 [x,y,z]，单位 g
 * @param  dt       : 采样时间间隔，单位 秒
 * @note   1kHz 调用；accel 应已做零偏校准
 */
void MahonyAHRS_Update(MahonyAHRS_t *maho , float gyro[3], float accel[3], float dt) ;

/**
 * @brief  从四元数计算欧拉角
 * @param  maho : 指向实例的指针
 * @param  e    : 输出欧拉角（单位：度）
 */
void MahonyAHRS_GetEulerAngles(MahonyAHRS_t *maho , EulerAngles_t *e);

/**
 * @brief  重置偏航角（保持当前 roll/pitch，把 yaw 归零）
 * @param  maho : 指向实例的指针
 * @note   用于重新建立朝向基准（如校准后）；转向环依赖 yaw 基准
 */
void MahonyAHRS_ResetYaw(MahonyAHRS_t *maho) ;

#endif
