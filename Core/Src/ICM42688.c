//
// Created by user on 2026/10/4.
//

#include "ICM42688.h"

#define WHO_AM_I            0x47
#define WHO_AM_I_ADDR       0x75
#define DATA12_ADDR         0x1F
#define PWR_MGMT0_ADDR      0x4E
#define ACCEL_CONFIG0_ADDR  0x50
#define ACCEL_ODR_1KHZ      0x06
#define GYRO_CONFIG0_ADDR   0x4F
#define GYRO_ODR_1KHZ       0x06
#define TEMPERATURE_ADDR    0x1D


static HAL_StatusTypeDef ICM42688_Read12Datas(imu_t *imu) {
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    uint8_t buffer_tx[13] = {0x80 | DATA12_ADDR , 0,0,0,0,0,0,0,0,0,0,0,0 };
    uint8_t buffer_rx[13] = {0};
    if ( HAL_OK != HAL_SPI_TransmitReceive(imu->hspi , buffer_tx , buffer_rx , 13 , HAL_TIMEOUT)) {
        HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET); return HAL_ERROR;
}
    for (uint8_t i = 0; i < 12; i++) {
        imu->buffer_temp[i] = buffer_rx[i + 1];
    }
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
    return HAL_OK;
}

static HAL_StatusTypeDef ICM42688_Trans(imu_t *imu) {
    imu->acx_temp = imu->buffer_temp[0] <<8 | imu->buffer_temp[1];
    imu->acy_temp = imu->buffer_temp[2] <<8 | imu->buffer_temp[3];
    imu->acz_temp = imu->buffer_temp[4] <<8 | imu->buffer_temp[5];
    imu->gyx_temp = imu->buffer_temp[6] <<8 | imu->buffer_temp[7];
    imu->gyy_temp = imu->buffer_temp[8] <<8 | imu->buffer_temp[9];
    imu->gyz_temp = imu->buffer_temp[10] <<8 | imu->buffer_temp[11];

    imu->acx = imu->acx_temp / imu->accel_sensitivity ;
    imu->acy = imu->acy_temp / imu->accel_sensitivity ;
    imu->acz = imu->acz_temp / imu->accel_sensitivity ;
    imu->gyx = imu->gyx_temp / imu->gyro_sensitivity ;
    imu->gyy = imu->gyy_temp / imu->gyro_sensitivity ;
    imu->gyz = imu->gyz_temp / imu->gyro_sensitivity ;

    return HAL_OK;
}



HAL_StatusTypeDef ICM42688_Init(imu_t *imu , int8_t acFSR , int16_t gyFSR) {
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    uint8_t buffer_tx[2] = {0x80 | WHO_AM_I_ADDR , 0 };
    uint8_t buffer_rx[2] = {0};
    if (HAL_SPI_TransmitReceive(imu->hspi , buffer_tx , buffer_rx, 2 , HAL_TIMEOUT) != HAL_OK) {
        HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);return HAL_ERROR;
}
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
    if (buffer_rx[1] != WHO_AM_I ) return HAL_ERROR;

    uint8_t pwr[2] = {PWR_MGMT0_ADDR & 0x7F, 0x0F};
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    if ( HAL_SPI_Transmit(imu->hspi, pwr, 2, HAL_TIMEOUT) != HAL_OK) {
        HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET); return HAL_ERROR;
}
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
    HAL_Delay(1);

    uint8_t accel_fs_bits;
    switch (acFSR) {
        case 2:  imu->accel_sensitivity = 32768.0f / 2.0f;  accel_fs_bits = 0x03; break;
        case 4:  imu->accel_sensitivity = 32768.0f / 4.0f;  accel_fs_bits = 0x02; break;
        case 8:  imu->accel_sensitivity = 32768.0f / 8.0f;  accel_fs_bits = 0x01; break;
        case 16: imu->accel_sensitivity = 32768.0f / 16.0f; accel_fs_bits = 0x00; break;
        default: return HAL_ERROR;
    }

    uint8_t accel_cfg[2] = {ACCEL_CONFIG0_ADDR & 0x7F,
                            (uint8_t)((accel_fs_bits << 5) | ACCEL_ODR_1KHZ)};
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    if (HAL_OK != HAL_SPI_Transmit(imu->hspi, accel_cfg, 2, HAL_TIMEOUT)) {
        HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);return HAL_ERROR;
}
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);

    uint8_t gyro_fs_bits;
    switch (gyFSR) {
        case 125:  imu->gyro_sensitivity = 32768.0f / 125.0f;  gyro_fs_bits = 0x04; break;
        case 250:  imu->gyro_sensitivity = 32768.0f / 250.0f;  gyro_fs_bits = 0x03; break;
        case 500:  imu->gyro_sensitivity = 32768.0f / 500.0f;  gyro_fs_bits = 0x02; break;
        case 1000: imu->gyro_sensitivity = 32768.0f / 1000.0f; gyro_fs_bits = 0x01; break;
        case 2000: imu->gyro_sensitivity = 32768.0f / 2000.0f; gyro_fs_bits = 0x00; break;
        default: return HAL_ERROR;
    }

    uint8_t gyro_cfg[2] = {GYRO_CONFIG0_ADDR & 0x7F,
                            (uint8_t)((gyro_fs_bits << 5) | GYRO_ODR_1KHZ)};
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    if (HAL_OK != HAL_SPI_Transmit(imu->hspi, gyro_cfg, 2, HAL_TIMEOUT)) {
        HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET); return HAL_ERROR;
}
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);


    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadSensorData(imu_t *imu) {
    if (ICM42688_Read12Datas(imu) != HAL_OK) return HAL_ERROR;
    if (ICM42688_Trans(imu) != HAL_OK) return HAL_ERROR;
    return HAL_OK;
}

HAL_StatusTypeDef ICM42688_ReadTemperature(imu_t *imu) {
    uint8_t buffer_tx[3] = {0x80 | TEMPERATURE_ADDR , 0 , 0};
    uint8_t buffer_rx[3] = {0};
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_RESET);
    if (HAL_SPI_TransmitReceive(imu->hspi, buffer_tx, buffer_rx, 3, HAL_TIMEOUT) != HAL_OK) {
        HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
        return HAL_ERROR;
    }
    HAL_GPIO_WritePin(imu->cs_port, imu->cs_pin, GPIO_PIN_SET);
    int16_t temp_raw = (int16_t)((buffer_rx[1] << 8) | buffer_rx[2]);
    imu->temperature_c = (float)temp_raw / 132.48f + 25.0f;
    return HAL_OK;
}
