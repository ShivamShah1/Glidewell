#include "error_handler.h"

/* UART port for serial communication if connected */
extern UART_HandleTypeDef huart2;

void Flash_BlinkBothLEDs(void) {
    for (int i = 0; i < 3; i++) {
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
        HAL_Delay(200);
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);
        HAL_Delay(200);
    }
}

/* Error displaying and handling */
void Handle_Error(const char *source, const char *message) {
    char buffer[128];
    snprintf(buffer, sizeof(buffer), "[ERROR] Source: %s | Message: %s\r\n", source, message);
    HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);

    HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_RESET); // Clear LEDs before setting a new pattern
    HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_RESET);

    if (strcmp(source, "IMU") == 0) {
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
    }
    else if (strcmp(source, "PPG") == 0) {
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
    }
    else if (strcmp(source, "BLE") == 0) {
        HAL_GPIO_WritePin(LED1_GPIO_PORT, LED1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(LED2_GPIO_PORT, LED2_PIN, GPIO_PIN_SET);
    }
    else if (strcmp(source, "Flash") == 0) {
        Flash_BlinkBothLEDs();
    }
}
