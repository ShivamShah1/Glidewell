#ifndef __FLASH_HANDLER_H
#define __FLASH_HANDLER_H

#include "main.h"

/* Flash memory settings */
#define FLASH_START_ADDR  0x08080000  // Start of custom flash sector (check linker script)
#define FLASH_PAGE_SIZE   0x800       // 2KB (STM32 typical flash page)
#define FLASH_END_ADDR    (FLASH_START_ADDR + (FLASH_PAGE_SIZE * 4)) // Reserve 4 pages

/* Total data structure */
typedef struct {
    uint32_t timestamp;
    int16_t acc_x;
    int16_t acc_y;
    int16_t acc_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
    uint32_t ppg_red1;
    uint32_t ppg_ir1;
    uint32_t ppg_red2;
    uint32_t ppg_ir2;
} SensorData_t;

bool Flash_Init(void);
bool Flash_EraseAll(void);
bool Flash_WriteData(const SensorData_t *data);
uint32_t Flash_GetNextAddress(void);
bool Flash_ReadData(uint32_t address, SensorData_t *data_out);

#endif // __FLASH_HANDLER_H
