//
// Created by user on 2026/10/5.
//
/**
 * @file  MahonyAHRS.c
 * @brief Mahony 互补滤波姿态解算实现
 *
 * 核心思想：
 *   陀螺仪（快、会漂）积分推进姿态；
 *   加速度计（稳、会晃）提供重力方向做基准；
 *   两者叉积得到姿态误差，用 Kp/Ki 反馈修正陀螺；
 *   用四元数表示姿态（避免万向锁），每步归一化（防漂移）。
 *
 * 数据流：gyro/accel/dt → Update（更新 q）→ GetEulerAngles（q → 欧拉角）
 */
#include "MahonyAHRS.h"
#include <math.h>

#define PI 3.14159265358979323846   /* 圆周率（<math.h> 也提供 M_PI，二选一） */

/* 1/sqrt(x)：归一化时算"模长的倒数"，比"先 sqrt 再除"少一次除法 */
static float inv_sqrt(float x) {
    return 1.0f / sqrtf(x);
}

void MahonyAHRS_Init(MahonyAHRS_t *maho , float kp , float ki ) {
    maho->kp = kp;
    maho->ki = ki;

    // 初始化积分误差项为0（陀螺零偏估计，从 0 开始在线累积）
    maho->integralFB[0] = 0.0f ;
    maho->integralFB[1] = 0.0f ;
    maho->integralFB[2] = 0.0f ;

    // 初始化姿态为单位四元数 (1,0,0,0) = 零旋转 = 水平静止
    maho->q[0] = 1.0f ;
    maho->q[1] = 0.0f ;
    maho->q[2] = 0.0f ;
    maho->q[3] = 0.0f ;

}

void MahonyAHRS_Update(MahonyAHRS_t *maho , float gyro[3], float accel[3], float dt) {
    /* ── 块1：角速度 度/秒 → 弧度/秒（四元数积分用弧度） ── */
    float gx = gyro[0] * (PI/180.0f) ;
    float gy = gyro[1] * (PI/180.0f) ;
    float gz = gyro[2] * (PI/180.0f) ;

    /* 块2：把当前四元数拷到局部变量（循环内反复用，放寄存器更快） */
    float q0 = maho->q[0], q1 = maho->q[1], q2 = maho->q[2], q3 = maho->q[3];

    float ax = accel[0], ay = accel[1], az = accel[2];
    float recipNorm;
    float halfvx, halfvy, halfvz;   /* 预测重力方向 */
    float halfex, halfey, halfez;   /* 叉积误差 */

    /* 块3：归一化加速度计（只关心重力方向，不关心大小；全 0 则跳过） */
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {
        // 归一化加速度计测量值 → 单位重力方向
        recipNorm = inv_sqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        /* 块4：从当前四元数"预测"重力方向（按我现在的姿态，重力应指向这） */
        halfvx = q1 * q3 - q0 * q2;
        halfvy = q0 * q1 + q2 * q3;
        halfvz = q0 * q0 - 0.5f + q3 * q3;

        /* 块5：叉积 = 误差（实测重力 × 预测重力；越不一致越大，方向=该绕哪轴修） */
        halfex = (ay * halfvz - az * halfvy);
        halfey = (az * halfvx - ax * halfvz);
        halfez = (ax * halfvy - ay * halfvx);

        /* 块6a：Ki 积分反馈——累计误差，稳态值≈陀螺零偏，在线补偿 */
        if(maho->ki > 0.0f) {
            maho->integralFB[0] += halfex * maho->ki * dt;
            maho->integralFB[1] += halfey * maho->ki * dt;
            maho->integralFB[2] += halfez * maho->ki * dt;
            gx += maho->integralFB[0]; // 应用积分补偿
            gy += maho->integralFB[1];
            gz += maho->integralFB[2];
        }

        /* 块6b：Kp 比例反馈——立即把姿态往正确方向拽（快） */
        gx += halfex * maho->kp;
        gy += halfey * maho->kp;
        gz += halfez * maho->kp;
    }

    /* 块7：四元数积分（一阶）：修正后的角速度 → 姿态变化 q̇ = 0.5·q⊗ω */
    q0 += (-q1 * gx - q2 * gy - q3 * gz) * 0.5f * dt;
    q1 += ( q0 * gx + q2 * gz - q3 * gy) * 0.5f * dt;
    q2 += ( q0 * gy - q1 * gz + q3 * gx) * 0.5f * dt;
    q3 += ( q0 * gz + q1 * gy - q2 * gx) * 0.5f * dt;

    /* 块8：归一化四元数（数值积分会偏离单位长度，强制拉回纯旋转） */
    recipNorm = inv_sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    maho->q[0] = q0 * recipNorm;
    maho->q[1] = q1 * recipNorm;
    maho->q[2] = q2 * recipNorm;
    maho->q[3] = q3 * recipNorm;
}

void MahonyAHRS_GetEulerAngles(MahonyAHRS_t *maho , EulerAngles_t *angles) {
    float q0 = maho->q[0], q1 = maho->q[1], q2 = maho->q[2], q3 = maho->q[3];

    // Roll（绕 x 轴）：标准四元数→欧拉角公式，atan2f 处理四象限
    float sinr_cosp = 2.0f * (q0 * q1 + q2 * q3);
    float cosr_cosp = 1.0f - 2.0f * (q1 * q1 + q2 * q2);
    angles->roll = atan2f(sinr_cosp, cosr_cosp) * (180.0f / M_PI);

    // Pitch（绕 y 轴）：asinf 在 |sinp|>=1 时无定义，需做 ±90° 奇点保护
    float sinp = 2.0f * (q0 * q2 - q3 * q1);
    if (fabsf(sinp) >= 1)
        angles->pitch = copysignf(M_PI / 2, sinp) * (180.0f / M_PI); // 超范围就给 ±90°
    else
        angles->pitch = asinf(sinp) * (180.0f / M_PI);

    // Yaw（绕 z 轴）
    float siny_cosp = 2.0f * (q0 * q3 + q1 * q2);
    float cosy_cosp = 1.0f - 2.0f * (q2 * q2 + q3 * q3);
    angles->yaw = atan2f(siny_cosp, cosy_cosp) * (180.0f / M_PI);
}

void MahonyAHRS_ResetYaw(MahonyAHRS_t *mf)
{
    /* 目标：保持当前 roll/pitch，把 yaw 归零 → 重建朝向基准（转向环用） */
    EulerAngles_t current_angles;
    MahonyAHRS_GetEulerAngles(mf, &current_angles);

    // 角度 度 → 弧度
    float roll_rad = current_angles.roll * M_PI / 180.0f;
    float pitch_rad = current_angles.pitch * M_PI / 180.0f;
    float yaw_rad = 0.0f; // 目标偏航角为0

    // 从欧拉角(roll, pitch, yaw=0)反推四元数
    float cy = cosf(yaw_rad * 0.5f);
    float sy = sinf(yaw_rad * 0.5f);
    float cp = cosf(pitch_rad * 0.5f);
    float sp = sinf(pitch_rad * 0.5f);
    float cr = cosf(roll_rad * 0.5f);
    float sr = sinf(roll_rad * 0.5f);

    // 写回四元数状态
    mf->q[0] = cr * cp * cy + sr * sp * sy;
    mf->q[1] = sr * cp * cy - cr * sp * sy;
    mf->q[2] = cr * sp * cy + sr * cp * sy;
    mf->q[3] = cr * cp * sy - sr * sp * cy;

    // 归一化（保持单位四元数）
    float norm = sqrtf(mf->q[0]*mf->q[0] + mf->q[1]*mf->q[1] + mf->q[2]*mf->q[2] + mf->q[3]*mf->q[3]);
    if (norm > 1e-6f) {
        mf->q[0] /= norm; mf->q[1] /= norm; mf->q[2] /= norm; mf->q[3] /= norm;
    }
}
