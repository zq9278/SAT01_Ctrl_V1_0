//
// Created by zq on 2025/5/21.
//
// log.c：串口日志模块（基于 FreeRTOS + UART + printf 重定向）

#include "LOG.h"
#include <stdarg.h>     // 用于处理可变参数 va_list
#include <stdio.h>      // vprintf
#include "uart_driver.h"
extern UartPort_t debug_uart_port;

void LOG(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;
    va_start(args, format);
    msg.len = vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);
    if (msg.len > LOG_BUF_LEN) msg.len = LOG_BUF_LEN; // 避免溢出
    xQueueSend(debug_uart_port.tx_queue, &msg, portMAX_DELAY);  // 任务态可阻塞
}

void LOG_ISR(const char *format, ...)
{
    LogMessage_t msg;
    va_list args;
    va_start(args, format);
    msg.len = vsnprintf(msg.buf, LOG_BUF_LEN, format, args);
    va_end(args);

    if (msg.len > LOG_BUF_LEN) msg.len = LOG_BUF_LEN;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(debug_uart_port.tx_queue, &msg, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


int __io_putchar(int ch)
{
    HAL_UART_Transmit(debug_uart_port.huart, (uint8_t *)&ch, 1,100);

    return ch;
}