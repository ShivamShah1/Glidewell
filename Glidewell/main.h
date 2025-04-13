#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "imu_driver/imu_driver.h"
#include "ppg_driver/ppg_driver.h"
#include "ble_handler/ble_handler.h"
#include "flash_handler/flash_handler.h"
#include "error_handler/error_handler.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/* Exported constants --------------------------------------------------------*/
#define IMU_TASK_PERIOD_MS    20    // 50 Hz
#define PPG_TASK_PERIOD_MS    10    // 100 Hz

#define PPG_QUEUE_SIZE 10  // Enough for burst + overflow
#define IMU_QUEUE_SIZE 10

#define WDT_TIMEOUT_MS 1000 // 1 second timeout

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* Peripheral handles --------------------------------------------------------*/
extern I2C_HandleTypeDef hi2c1;
extern UART_HandleTypeDef huart2;

/* Task handles --------------------------------------------------------------*/
extern osThreadId_t imuTaskHandle;
extern osThreadId_t ppgTaskHandle;
extern osThreadId_t bleTaskHandle;

/* Task Implementations */
void StartIMUTask(void *argument) {
    IMU_Data_t imu_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(IMU_TASK_PERIOD_MS);
  
    for (;;) {
      if (osMutexAcquire(i2cMutexHandle, osWaitForever) == osOK){
          if (IMU_Read(&imu_data) != HAL_OK) {
              Handle_Error("IMU", "Failed to read IMU data");
          }
          else {
              osMessageQueuePut(imuQueueHandle, &imu_data, 0, 0);
          }
          osMutexRelease(i2cMutexHandle);
      }
  
      // Reset the watchdog timer to prevent system reset
      HAL_IWDG_Refresh(&hiwdg);
  
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void StartPPGTask(void *argument) {
    PPG_Data_t ppg_data;
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(PPG_TASK_PERIOD_MS);
  
    for (;;) {
      if (osMutexAcquire(i2cMutexHandle, osWaitForever) == osOK) {
          if (PPG_Read(&ppg_data) != HAL_OK) {
              Handle_Error("PPG", "Failed to read PPG data");
          } else {
              osMessageQueuePut(ppgQueueHandle, &ppg_data, 0, 0);
          }
          osMutexRelease(i2cMutexHandle);
      }
  
      // Reset the watchdog timer to prevent system reset
      HAL_IWDG_Refresh(&hiwdg);
  
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void StartBLETask(void *argument) {
    for (;;) {
      if (BLE_Process() != HAL_OK) {
          Handle_Error("BLE", "BLE processing error");
        }
  
      // Reset the watchdog timer to prevent system reset
      HAL_IWDG_Refresh(&hiwdg);
      
      osDelay(100);
    }
}
  
void StartFlashTask(void *argument) {
      IMU_Data_t imu_data;
      PPG_Data_t ppg_data1, ppg_data2;
      SensorData_t sensor_data;
    
      for (;;) {
        if (logging_enabled) {
          // 1. Get 1 IMU sample (50 Hz)
          if (osMessageQueueGet(imuQueueHandle, &imu_data, 0, 0) == osOK) {
              // 2. Get 2 PPG samples (100 Hz)
              if (osMessageQueueGet(ppgQueueHandle, &ppg_data1, 0, 0) == osOK &&
              osMessageQueueGet(ppgQueueHandle, &ppg_data2, 0, 0) == osOK) {
                  sensor_data.timestamp = HAL_GetTick();  // Assign current timestamp
          
                  // Fill IMU fields
                  sensor_data.acc_x = imu_data.acc_x;
                  sensor_data.acc_y = imu_data.acc_y;
                  sensor_data.acc_z = imu_data.acc_z;
                  sensor_data.gyro_x = imu_data.gyro_x;
                  sensor_data.gyro_y = imu_data.gyro_y;
                  sensor_data.gyro_z = imu_data.gyro_z;
          
                  // Fill PPG1 fields
                  sensor_data.ppg_red1 = ppg_data1.red;
                  sensor_data.ppg_ir1 = ppg_data1.ir;
                  // Fill PPG1 fields
                  sensor_data.ppg_red2 = ppg_data2.red;
                  sensor_data.ppg_ir2 = ppg_data2.ir;
          
                  // Write to flash
                  if (!Flash_WriteData(&sensor_data)) {
                      Handle_Error("Flash", "Failed to write sensor data");
                  }
              }
          }
          osDelay(1);
          }
  
          // Reset the watchdog timer to prevent system reset
          HAL_IWDG_Refresh(&hiwdg);
      }
} 
  
/* Peripheral Initialization Functions */
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
    // Enable LSI oscillator for the watchdog timer
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 10;
    RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
    RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
    RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
    HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                                | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4);
}
  
void MX_GPIO_Init(void)
  {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
  
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
  
    /* Configure PA5 (LED1) */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  
    /* Configure LED2 GPIO (PB14) */
      GPIO_InitStruct.Pin = GPIO_PIN_14;
      GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
      GPIO_InitStruct.Pull = GPIO_NOPULL;
      GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
      HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  
    /* Turn off both LEDs initially */
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
}
  
void MX_I2C1_Init(void) {
    hi2c1.Instance = I2C1;
    hi2c1.Init.ClockSpeed = 100000;
    hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c1);
}
  
void MX_USART2_UART_Init(void) {
    huart2.Instance = USART2;
    huart2.Init.BaudRate = 115200;
    huart2.Init.WordLength = UART_WORDLENGTH_8B;
    huart2.Init.StopBits = UART_STOPBITS_1;
    huart2.Init.Parity = UART_PARITY_NONE;
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    HAL_UART_Init(&huart2);
}
  
void WDT_Init(void) {
      // Enable LSI (Low-Speed Internal) oscillator (required for IWDG)
      RCC_OscInitTypeDef RCC_OscInitStruct = {0};
      RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
      RCC_OscInitStruct.LSIState = RCC_LSI_ON;
      HAL_RCC_OscConfig(&RCC_OscInitStruct);
      
      // Enable the IWDG (Independent Watchdog)
      IWDG_HandleTypeDef hiwdg;
      hiwdg.Instance = IWDG;
      hiwdg.Init.Prescaler = IWDG_PRESCALER_64;    // Prescaler value to adjust timeout
      hiwdg.Init.Reload = (WDT_TIMEOUT_MS * (SystemCoreClock / 1000)) / 64;  // Timeout period calculation
      
      if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
          Handle_Error("WDT", "Failed to initialize watchdog timer");
      }
}

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
