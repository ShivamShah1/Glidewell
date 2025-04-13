#ifndef __PPG_DRIVER_H
#define __PPG_DRIVER_H

#include "main.h"

/* I2C address for MAX30102 */
#define MAX30102_ADDR  (0x57 << 1)  // 7-bit address + HAL shift

typedef struct {
    uint32_t red;
    uint32_t ir;
} PPG_Data_t;

/* PPG Functions */
bool PPG_Init(I2C_HandleTypeDef *hi2c);
bool PPG_Read(PPG_Data_t *data);

#endif // __PPG_DRIVER_H
