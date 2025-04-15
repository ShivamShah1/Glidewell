#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "usart.h"
#include <string.h>
#include <stdio.h>

/* Optional LED indicator (customize to your board) */
#define LED1_GPIO_PORT GPIOA
#define LED1_PIN       GPIO_PIN_5
#define LED2_GPIO_PORT GPIOB
#define LED2_PIN       GPIO_PIN_14

void Handle_Error(const char *source, const char *message);

#endif // ERROR_HANDLER_H
