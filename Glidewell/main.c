/*
    this is freeRTOS simulation using STM32x board series with cmsis os
    for collecting 2 sensor data (PPG and IMU via I2C) with a periodicity of 
    50Hz and 100Hz respectively. The user can connect with the STM board
    with ble connection. A nordic ble module is connected to the board via 
    UART connection, and after connecting to board via ble the user can send
    various commands like to store the sensor datta in the flash memory,
    browse the data and download it. I have also connected 2 led light for 
    visual display is any error occurs and also provided error handling methods.

    At the start fo the board, I have initialized only required ports, timers, ble and
    most importanlty WDT(watchdog timer with reset time of 1 sec). I have alloted 
    4 threads for imu sensor, ppg sensor, ble connection detector and flash storager.
    I have implemented priority based schedulling algorithm and this is the chart for 
    the priority ppg sensor -> imu sensor -> flash storage -> ble connection. To 
    store the data, I have used messaged queues as buffers. I have also provided mutex
    for the i2c bus allocation.

    Author: Shivam Shah
*/

#include "main.h"

/* Private function prototypes for initiallizations */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART2_UART_Init(void);

/* FreeRTOS task prototypes */
void StartIMUTask(void *argument);
void StartPPGTask(void *argument);
void StartBLETask(void *argument);
void StartFlashTask(void *argument);

/* Global handles */
/* Threads */
osThreadId_t imuTaskHandle, ppgTaskHandle, bleTaskHandle, flashTaskHandle;
/* Messaging Queues (IPC) for data communication between different processes */
osMessageQueueId_t imuQueueHandle, ppgQueueHandle;
/* mutex for i2c bus access */
osMutexId_t i2cMutexHandle;

/* Define the mutex attributes for helping priority inheritance */
const osMutexAttr_t i2cMutexAttr = {
    .name = "I2CMutex",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

/* I2C and UART driver initialization */
I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

/* Global variables for logging in flash memory */
volatile bool logging_enabled = false;
volatile bool ble_connected = false;

int main(void)
{
  /* MCU Configuration */
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART2_UART_Init();

  /* Init sensor/ble drivers */
  IMU_Init(&hi2c1);
  PPG_Init(&hi2c1);
  BLE_Init(&huart2);
  Flash_Init();

  /* Init RTOS */
  osKernelInitialize();

  /* mutex initializatino for i2c */
  i2cMutexHandle = osMutexNew(&i2cMutexAttr);

  /* Create message queues */
  imuQueueHandle = osMessageQueueNew(IMU_QUEUE_SIZE, sizeof(IMU_Data_t), NULL);
  ppgQueueHandle = osMessageQueueNew(PPG_QUEUE_SIZE, sizeof(PPG_Data_t), NULL);

  /* Create tasks */
  const osThreadAttr_t imuTaskAttr = { .name = "IMUTask", .priority = osPriorityAboveNormal, .stack_size = 512 };
  imuTaskHandle = osThreadNew(StartIMUTask, NULL, &imuTaskAttr);

  const osThreadAttr_t ppgTaskAttr = { .name = "PPGTask", .priority = osPriorityHigh, .stack_size = 512 };
  ppgTaskHandle = osThreadNew(StartPPGTask, NULL, &ppgTaskAttr);

  const osThreadAttr_t bleTaskAttr = { .name = "BLETask", .priority = osPriorityLow, .stack_size = 512 };
  bleTaskHandle = osThreadNew(StartBLETask, NULL, &bleTaskAttr);

  const osThreadAttr_t flashTaskAttr = { .name = "FlashTask", .priority = osPriorityNormal, .stack_size = 512 };
  flashTaskHandle = osThreadNew(StartFlashTask, NULL, &flashTaskAttr);

  osKernelStart();
}

/* IMU Task Initializer ----------------------------------------------------*/
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

/* PPG Task Initializer ------------------------------------------------------*/
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

/* BLE Task Initializer -------------------------------------------------------*/
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

/* Flash Task Initializer -------------------------------------------------------*/
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