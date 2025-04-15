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

/*  Peripheral Initialization Functions --------------------------------------------------*/
void SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
    // Enable LSI oscillator for the watchdog timer
    WDT_Init();
  
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

/* GPIO Pin Initializer -------------------------------------------------------*/
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

/* I2C Initializer -------------------------------------------------------*/
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

/* USART Initializer -------------------------------------------------------*/
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

/* Watchdog Timer Initializer -------------------------------------------------------*/
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
