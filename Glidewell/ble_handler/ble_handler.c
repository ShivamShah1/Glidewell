#include "ble_handler.h"

volatile bool ble_connected = false;
volatile bool logging_enabled = false;

static uint8_t ble_tx_buffer[256];

/* this is called by cmsis os when the BLE stack when a connection is established */
void BLE_ConnectedCallback(void) {
    ble_connected = true;
}

/* this is called by cmsis os when the BLE stack when a disconnection occurs */
void BLE_DisconnectedCallback(void) {
    ble_connected = false;
}

/* this is called by cmsis os when a request is received from BLE central */
void Custom_APP_Notification(Custom_App_Notification_evt_t *pNotification)
{
    switch(pNotification->EvtOpcode) {
        case CUSTOM_STM_WRITE_EVT:
            memset(last_received_command, 0, MAX_BLE_COMMAND_LEN);
            strncpy(last_received_command, (const char *)pNotification->DataTransfered.pPayload, MAX_BLE_COMMAND_LEN - 1);
            command_available = true;  // Set flag to let BLE Task know
            break;
        default:
            break;
    }
}

/* To initialize the ble at the start of the board */
void BLE_Init(void)
{   
    int retry_count = 0;
    HAL_StatusTypeDef ble_status;

    do {
        ble_status = APP_BLE_Init();  // This initializes everything related to BLE, including HCI, GATT, GAP, and custom services
        if (ble_status == HAL_OK) break;
        retry_count++;
    } while (retry_count < MAX_RETRY);
    if(ble_status!=HAL_OK){
        Handle_Error("BLE", "Failed to init ble");
        osThreadSuspend(NULL);  // Suspend task to avoid corrupted loop
        NVIC_SystemReset() // software reset (soft wdt)
    }
}

/* to check the ble status */
bool BLE_status(void) {
    return ble_connected;
}

/* to communicate through ble */
void BLE_Gatt(const char* msg)
{
    uint16_t len = strlen(msg);
    if (len > sizeof(ble_tx_buffer)) len = sizeof(ble_tx_buffer);
    memcpy(ble_tx_buffer, msg, len);
    // this is used when we want to send the data through the ble to central
    if(Custom_STM_App_Update_Char(CUSTOM_STM_NOTIFY_CHAR_UUID, ble_tx_buffer) != HAL_OK){
        Handle_Error("BLE", "Failed to init ble");
        osThreadSuspend(NULL);  // Suspend task to avoid corrupted loop
        NVIC_SystemReset() // software reset (soft wdt)
    }
}

/* process the user cmd */
void BLE_ProcessCommand(const char *cmd)
{
    if (strstr(cmd, BLE_CMD_START_LOGGING)) {
        logging_enabled = true;
        BLE_Gatt("Logging started\n");
    }
    else if (strstr(cmd, BLE_CMD_STOP_LOGGING)) {
        logging_enabled = false;
        BLE_Gatt("Logging stopped\n");
    }
    else if (strstr(cmd, BLE_CMD_BROWSE_LOGS)) {
        char msg[64];
        uint32_t entries = (Flash_GetNextAddress() - FLASH_START_ADDR) / sizeof(SensorData_t);
        snprintf(msg, sizeof(msg), "Log Entries: %lu\n", entries);
        BLE_Gatt(msg);
    }
    else if (strstr(cmd, BLE_CMD_DOWNLOAD_LOG)) {
        char msg[128];
        SensorData_t data;
        uint32_t addr = FLASH_START_ADDR;
        uint32_t end = Flash_GetNextAddress();

        while (addr < end) {
            if (Flash_ReadData(addr, &data)) {
                snprintf(msg, sizeof(msg),
                    "T:%lu AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d RED1:%lu IR1:%lu RED2:%lu IR2:%lu\n",
                    data.timestamp, data.acc_x, data.acc_y, data.acc_z,
                    data.gyro_x, data.gyro_y, data.gyro_z,
                    data.ppg_red1, data.ppg_ir1, data.ppg_red2, data.ppg_ir2);
                BLE_Gatt(msg);
            }
            addr += sizeof(SensorData_t);
        }
    }
}
