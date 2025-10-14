// Created by zq on 2025/5/21.

#include "AppMain.h"
#include "SYS.h"
#include "uart_driver.h"
#include "user_uart_hal.h"
#include "Uart_Communicate.h"


extern UART_HandleTypeDef huart1;
UartPort_t uart1_port;

void Main_App(void)
{
    Uart_Init(&uart1_port, &huart1, 115200);
    float temp = 36.5f;
    Uart_SendFrame(&uart1_port, DATA_FLOAT, FLOAT_TEMPERATURE, &temp);
    while (1) {
        HAL_Delay(100);
        uint8_t b;
        if (Uart_ReadByte(&uart1_port, &b)) {
            // 从环形缓冲里取数据
        }
    }
}