#include "flash_handler.h"

static uint32_t current_flash_addr = FLASH_START_ADDR;

/* initializing the flash memory for logging */
bool Flash_Init(void)
{
    HAL_FLASH_Unlock();
    current_flash_addr = Flash_GetNextAddress();
    HAL_FLASH_Lock();
    return true;
}

/* erasing the existing sensor memory only */
bool Flash_EraseAll(void)
{
    FLASH_EraseInitTypeDef erase;
    uint32_t page_error;

    HAL_FLASH_Unlock();

    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Sector = FLASH_SECTOR_7; // Make sure this sector is not used by program
    erase.NbSectors = 1;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase, &page_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }

    current_flash_addr = FLASH_START_ADDR;
    HAL_FLASH_Lock();
    return true;
}

/* writing to the flash memory */
bool Flash_WriteData(const SensorData_t *data)
{
    HAL_FLASH_Unlock();

    const uint32_t *data_words = (const uint32_t *)data;
    uint32_t data_len = sizeof(SensorData_t) / 4;

    for (uint32_t i = 0; i < data_len; i++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, current_flash_addr, data_words[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
        current_flash_addr += 4;
    }

    HAL_FLASH_Lock();
    return true;
}

/* reading from the flash memory */
bool Flash_ReadData(uint32_t address, SensorData_t *data_out)
{
    if (address < FLASH_START_ADDR || address >= FLASH_END_ADDR)
        return false;

    const SensorData_t *stored = (const SensorData_t *)address;
    *data_out = *stored;
    return true;
}

/* get the next available flash space to store the data */
uint32_t Flash_GetNextAddress(void)
{
    uint32_t addr = FLASH_START_ADDR;
    SensorData_t check;

    while (addr + sizeof(SensorData_t) <= FLASH_END_ADDR) {
        const SensorData_t *entry = (SensorData_t *)addr;
        if (entry->timestamp == 0xFFFFFFFF) {
            return addr;
        }
        addr += sizeof(SensorData_t);
    }

    return FLASH_START_ADDR;  // If full, wrap around (or handle with flag)
}
