#ifndef __BLE_HANDLER_H
#define __BLE_HANDLER_H

#include "main.h"
#include "FreeRTOS/bluetooth/bluetooth.h"
#include "FreeRTOS/bluetooth/gatt.h"

#define BLE_RX_BUFFER_SIZE  128
#define BLE_TX_BUFFER_SIZE  128

/* BLE Commands */
#define BLE_CMD_START_LOGGING    "START_LOG"
#define BLE_CMD_STOP_LOGGING     "STOP_LOG"
#define BLE_CMD_BROWSE_LOGS      "BROWSE_LOGS"
#define BLE_CMD_DOWNLOAD_LOG     "DOWNLOAD_LOG"

/* BLE State Flags */
extern volatile bool ble_connected;
extern volatile bool logging_enabled;

/* BLE Functions */
void BLE_Init(void);
void BLE_status(void);
void BLE_ProcessCommand(const char *cmd);

#endif // __BLE_HANDLER_H
