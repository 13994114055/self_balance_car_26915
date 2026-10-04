#ifndef __ICM42688_H
#define __ICM42688_H

#include "spi.h"
#include "main.h"

/**
 * @file   ICM42688.h
 * @brief  ICM42688P 六轴 IMU 驱动接口（SPI）
 */

/**
 * @brief ICM42688 实例句柄：硬件绑定 + 数据 + 换算系数
 */
typedef struct {
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    int16_t acx_temp , acy_temp , acz_temp , gyx_temp , gyy_temp , gyz_temp;
    float accle[3];         /* 加速度 (g)，X/Y/Z */
    float gyro[3];         /* 角速度 (°/s)，X/Y/Z */

    float accel_sensitivity;     /* 加速度灵敏度 = 32768/量程 (LSB/g) */
    float gyro_sensitivity;      /* 陀螺仪灵敏度 = 32768/量程 (LSB/dps) */
    float temperature_c;
    uint8_t buffer_temp[12];
}imu_t;

/**
 * @brief  初始化 ICM42688：校验 WHO_AM_I、配置电源/量程/采样率，并算好灵敏度
 * @param  imu   : 指向 IMU 实例的指针（含 SPI/CS 硬件绑定）
 * @param  acFSR : 加速度计量程（ACCEL_FS_xxx，如 ACCEL_FS_8G）
 * @param  gyFSR : 陀螺仪量程（GYRO_FS_xxx，如 GYRO_FS_500DPS）
 * @return HAL_OK 成功；HAL_ERROR 失败（WHO_AM_I 不符 / 参数非法）
 * @note   必须在 ReadSensorData 之前调用；灵敏度在此一次性算好
 */
HAL_StatusTypeDef ICM42688_Init(imu_t *imu , int8_t acFSR , int16_t gyFSR);

/**
 * @brief  读取三轴加速度和三轴陀螺仪，换算后存入 imu 实例
 * @param  imu : 指向 IMU 实例的指针
 * @return HAL_OK 成功；HAL_ERROR 失败
 * @note   1kHz 调用；一次突发读 12 字节；结果单位：加速度 g、角速度 °/s
 */
HAL_StatusTypeDef ICM42688_ReadSensorData(imu_t *imu);

/**
 * @brief  读取 ICM42688 的芯片温度
 * @param  imu : 指向 IMU 实例的指针
 * @return HAL_OK 成功；HAL_ERROR 失败
 */
HAL_StatusTypeDef ICM42688_ReadTemperature(imu_t *imu);

#endif