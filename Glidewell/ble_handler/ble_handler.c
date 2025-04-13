#include "ble_handler.h"

extern UART_HandleTypeDef huart1; // UART for BLE
volatile bool ble_connected = false;
volatile bool logging_enabled = false;

static char ble_rx_buffer[BLE_RX_BUFFER_SIZE];
static uint16_t rx_index = 0;

void BLE_Init(void)
{
    // Start receiving BLE data via interrupt
    HAL_UART_Receive_IT(&huart1, (uint8_t *)&ble_rx_buffer[rx_index], 1);
}

void BLE_Task(void *argument)
{
    BLE_Init();
    for (;;)
    {
        osDelay(100); // Poll BLE status every 100 ms

        // Placeholder: Update this to reflect real connection check
        ble_connected = true;
    }
}

void BLE_Send(const char *msg)
{
    HAL_UART_Transmit(&huart1, (uint8_t *)msg, strlen(msg), HAL_MAX_DELAY);
    HAL_UART_Transmit(&huart1, (uint8_t *)"\r\n", 2, HAL_MAX_DELAY); // Newline for readability
}

void BLE_ProcessCommand(const char *cmd)
{
    if (strstr(cmd, BLE_CMD_START_LOGGING))
    {
        logging_enabled = true;
        BLE_Send("Logging Started");
    }
    else if (strstr(cmd, BLE_CMD_STOP_LOGGING))
    {
        logging_enabled = false;
        BLE_Send("Logging Stopped");
    }
    else if (strstr(cmd, BLE_CMD_BROWSE_LOGS))
    {
        char msg[64];
        uint32_t entries = (Flash_GetNextAddress() - FLASH_START_ADDR) / sizeof(SensorData_t);
        snprintf(msg, sizeof(msg), "Log Entries: %lu", entries);
        BLE_Send(msg);
    }
    else if (strstr(cmd, BLE_CMD_DOWNLOAD_LOG))
    {
        char msg[128];
        SensorData_t data;
        uint32_t addr = cmd.start_address();
        uint32_t end = cmd.end_address();

        BLE_Send("Start of Log:");

        while (addr < end) {
            if (Flash_ReadData(addr, &data)) {
                snprintf(msg, sizeof(msg),
                    "T:%lu AX:%d AY:%d AZ:%d GX:%d GY:%d GZ:%d RED1:%lu IR1:%lu RED2:%lu IR2:%lu",
                    data.timestamp, data.acc_x, data.acc_y, data.acc_z,
                    data.gyro_x, data.gyro_y, data.gyro_z,
                    data.ppg_red1, data.ppg_ir1, data.ppg_red2, data.ppg_ir2
                );
                BLE_Send(msg);
            }
            addr += sizeof(SensorData_t);
        }

        BLE_Send("End of Log");
    }
    else
    {
        BLE_Send("Unknown command");
    }
}

/* UART RX Complete Callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (ble_rx_buffer[rx_index] == '\n' || ble_rx_buffer[rx_index] == '\r')
        {
            ble_rx_buffer[rx_index] = '\0';
            BLE_ProcessCommand(ble_rx_buffer);
            rx_index = 0;
        }
        else
        {
            rx_index++;
            if (rx_index >= BLE_RX_BUFFER_SIZE) rx_index = 0;
        }

        // Continue receiving
        HAL_UART_Receive_IT(&huart1, (uint8_t *)&ble_rx_buffer[rx_index], 1);
    }
}
