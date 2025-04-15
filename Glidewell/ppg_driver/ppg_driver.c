#include "ppg_driver.h"

/* initalizing the i2c port for the ppg sensor */
static I2C_HandleTypeDef *ppg_i2c = NULL;

/* setting the ppg sensor */
static bool MAX30102_WriteReg(uint8_t reg, uint8_t value)
{
    uint8_t data[2] = {reg, value};
    return HAL_I2C_Master_Transmit(ppg_i2c, MAX30102_ADDR, data, 2, HAL_MAX_DELAY) == HAL_OK;
}

/* reading from the sensor */
static bool MAX30102_ReadFIFO(uint8_t *buffer, uint8_t length)
{
    uint8_t reg = 0x07;  // FIFO_DATA register
    if (HAL_I2C_Master_Transmit(ppg_i2c, MAX30102_ADDR, &reg, 1, HAL_MAX_DELAY) != HAL_OK)
        return false;

    return HAL_I2C_Master_Receive(ppg_i2c, MAX30102_ADDR, buffer, length, HAL_MAX_DELAY) == HAL_OK;
}

/* initializing the sensor */
bool PPG_Init(I2C_HandleTypeDef *hi2c)
{
    ppg_i2c = hi2c;

    // Reset the device
    if (!MAX30102_WriteReg(0x09, 0x40))  // Mode config: reset
        return false;

    // Initialize registers (standard config)
    if (!MAX30102_WriteReg(0x09, 0x03))  // SpO2 mode
        return false;
    if (!MAX30102_WriteReg(0x0A, 0x27))  // SpO2 config: 411 Hz, 16-bit ADC
        return false;
    if (!MAX30102_WriteReg(0x0C, 0x24))  // LED pulse amplitude
        return false;

    return true;
}

/* reading and storing the data from the sensor */
bool PPG_Read(PPG_Data_t *data)
{
    uint8_t fifo_data[6];

    if (!MAX30102_ReadFIFO(fifo_data, 6))
        return false;

    // IR and RED values are 18-bit in 3 bytes each
    data->red = ((uint32_t)(fifo_data[0] & 0x03) << 16) | ((uint32_t)fifo_data[1] << 8) | fifo_data[2];
    data->ir  = ((uint32_t)(fifo_data[3] & 0x03) << 16) | ((uint32_t)fifo_data[4] << 8) | fifo_data[5];

    return true;
}
