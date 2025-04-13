#ifndef __IMU_DRIVER_H
#define __IMU_DRIVER_H

#include "main.h"

typedef struct {
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
} IMU_Data_t;

/* I2C address for MPU6050 */
#define MPU6050_ADDR           0x68 << 1  // Shifted for HAL (7-bit address + R/W bit)

/* MPU6050 Registers */
#define MPU6050_REG_PWR_MGMT_1 0x6B
#define MPU6050_REG_ACCEL_XOUT_H 0x3B
#define MPU6050_REG_GYRO_XOUT_H  0x43

/* IMU Functions */
bool IMU_Init(I2C_HandleTypeDef *hi2c);
bool IMU_Read(IMU_Data_t *data);

#endif // __IMU_DRIVER_H
