#include "ble_handler.h"

ble_connected = false;
logging_enabled = false;

static uint8_t ble_tx_buffer[256];

// Initialize BLE and start services
void BLE_Init(void) {
    if (BLE_Enable() != HAL_OK) {
        Handle_Error("BLE", "Failed to initialize BLE");
        return;
    }
    BLE_Advertise();
}

bool BLE_status(void){
    ble_connected = BLE_Conneted();
    return ble_connected;
}

/* process the ble command received */
void BLE_ProcessCommand(const char *cmd)
{
    if (strstr(cmd, BLE_CMD_START_LOGGING))
    {
        logging_enabled = true;
    }
    else if (strstr(cmd, BLE_CMD_STOP_LOGGING))
    {
        logging_enabled = false;
    }
    else if (strstr(cmd, BLE_CMD_BROWSE_LOGS))
    {
        char msg[64];
        uint32_t entries = (Flash_GetNextAddress() - FLASH_START_ADDR) / sizeof(SensorData_t);
        snprintf(msg, sizeof(msg), "Log Entries: %lu", entries);
        BLE_Gatt(msg);
    }
    else if (strstr(cmd, BLE_CMD_DOWNLOAD_LOG))
    {
        char msg[128];
        SensorData_t data;
        uint32_t addr = cmd.start_address();
        uint32_t end = cmd.end_address();

        while (addr < end) {
            if (Flash_ReadData(addr, &data)) {
                snprintf(msg, sizeof(msg),
                    "T:%lu AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d RED1:%lu IR1:%lu RED2:%lu IR2:%lu",
                    data.timestamp, data.acc_x, data.acc_y, data.acc_z,
                    data.gyro_x, data.gyro_y, data.gyro_z,
                    data.ppg_red1, data.ppg_ir1, data.ppg_red2, data.ppg_ir2
                );
                BLE_Gatt(msg);
            }
            addr += sizeof(SensorData_t);
        }
    }
    else return;
}