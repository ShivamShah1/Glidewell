#include "main.h"

/* Private function prototypes */
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
osThreadId_t imuTaskHandle, ppgTaskHandle, bleTaskHandle, flashTaskHandle;
osMessageQueueId_t imuQueueHandle, ppgQueueHandle;
osMutexId_t i2cMutexHandle;

/* Define the mutex attributes */
const osMutexAttr_t i2cMutexAttr = {
    .name = "I2CMutex",
    .attr_bits = osMutexRecursive,
    .cb_mem = NULL,
    .cb_size = 0
};

I2C_HandleTypeDef hi2c1;
UART_HandleTypeDef huart2;

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

  const osThreadAttr_t bleTaskAttr = { .name = "BLETask", .priority = osPriorityNormal, .stack_size = 512 };
  bleTaskHandle = osThreadNew(StartBLETask, NULL, &bleTaskAttr);

  const osThreadAttr_t flashTaskAttr = { .name = "FlashTask", .priority = osPriorityLow, .stack_size = 512 };
  flashTaskHandle = osThreadNew(StartFlashTask, NULL, &flashTaskAttr);

  osKernelStart();
}