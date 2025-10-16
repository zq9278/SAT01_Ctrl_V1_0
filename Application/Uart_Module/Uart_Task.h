//
// Created by zq on 25-9-11.
//

#ifndef UART_TASK_H
#define UART_TASK_H

#include "FreeRTOS.h"
#include "semphr.h"
#include "queue.h"
#include "uart_driver.h"

// UART TX DMA scheduling task
void SerialTxTask(void *argument);

// UART RX/LOG handling task
void UartRxParserTask(void *argument);

#endif // UART_TASK_H

