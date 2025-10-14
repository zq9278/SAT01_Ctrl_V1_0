#ifndef USER_UART_HAL_H
#define USER_UART_HAL_H

#include <stdint.h>

/* 
 * 用户必须在移植时提供以下函数
 * instance 可以是 USART1 基地址 / SDK 句柄 / 用户自定义结构体
 */

/* 初始化硬件 UART */
void UART_HAL_Init(void *instance, uint32_t baudrate);

/* 发送一个字节（阻塞即可） */
void UART_HAL_SendByte(void *instance, uint8_t byte);

/* 使能接收中断（收到字节时应调用 UART_Driver_OnRxByte） */
void UART_HAL_EnableRxInterrupt(void *instance);

#endif