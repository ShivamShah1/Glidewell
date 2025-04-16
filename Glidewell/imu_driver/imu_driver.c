#include "imu_driver.h"

/* i2c connection for imu */
static I2C_HandleTypeDef *imu_i2c = NULL;

/* converting the data to a single int16_t */
static int16_t ConvertToInt16(uint8_t high, uint8_t low)
{
    return ((int16_t)high << 8) | low;
}

/* initialinzing the i2c port for the sensor */
bool IMU_Init(I2C_HandleTypeDef *hi2c)
{
    imu_i2c = hi2c;

    uint8_t data[2] = {MPU6050_REG_PWR_MGMT_1, 0x00}; // Wake up the MPU6050
    if (HAL_I2C_Master_Transmit(imu_i2c, MPU6050_ADDR, data, 2, HAL_MAX_DELAY) != HAL_OK)
        return false;

    HAL_Delay(100); // Let IMU stabilize
    return true;
}

/* reading the data from the imu */
bool IMU_Read(IMU_Data_t *data)
{
    uint8_t raw_data[14];
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;

    // Read 14 bytes: Accel (6), Temp (2), Gyro (6)
    if (HAL_I2C_Master_Transmit(imu_i2c, MPU6050_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;

    if (HAL_I2C_Master_Receive(imu_i2c, MPU6050_ADDR, raw_data, 14, HAL_MAX_DELAY) != HAL_OK)
        return false;

    // Convert accelerometer data
    data->acc_x = ConvertToInt16(raw_data[0], raw_data[1]) / 16384.0f;  // Assuming default sensitivity
    data->acc_y = ConvertToInt16(raw_data[2], raw_data[3]) / 16384.0f;
    data->acc_z = ConvertToInt16(raw_data[4], raw_data[5]) / 16384.0f;

    // Convert gyroscope data
    data->gyro_x = ConvertToInt16(raw_data[8], raw_data[9]) / 131.0f;   // Assuming default sensitivity
    data->gyro_y = ConvertToInt16(raw_data[10], raw_data[11]) / 131.0f;
    data->gyro_z = ConvertToInt16(raw_data[12], raw_data[13]) / 131.0f;

    return true;
}
