//
// Created by zq on 25-9-11.
//

#include "Uart_Task.h"
#include "uart_driver.h"
#include "LOG.h"
extern UartPort_t rk3576_uart_port;
extern UartPort_t debug_uart_port;


// TX DMA scheduling for rk3576 port
void SerialTxTask(void *argument)
{
    (void)argument;
    UartTxMessage_t msg;
    for (;;)
    {
        if (xSemaphoreTake(rk3576_uart_port.uartTxDoneSem, portMAX_DELAY) == pdTRUE)
        {
            if (xQueueReceive(rk3576_uart_port.tx_queue, &msg, portMAX_DELAY) == pdPASS)
            {
                HAL_UART_Transmit_DMA(rk3576_uart_port.huart, (uint8_t *)msg.data, msg.length);
            }
            else
            {
                xSemaphoreGive(rk3576_uart_port.uartTxDoneSem);
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    }
}

// RX parsing for rk3576 + direct log output on debug port
void UartRxParserTask(void *argument)
{
    (void)argument;
    UartRxMessage_t rx_msg;
    LogMessage_t log_msg;

    for (;;)
    {
        if (xQueueReceive(rk3576_uart_port.rx_queue, &rx_msg, 0) == pdPASS)
        {
            rk3576_uart_port.parser(rx_msg.data, rx_msg.length);
        }

        if (xQueueReceive(debug_uart_port.tx_queue, &log_msg, 0) == pdPASS)
        {
            HAL_UART_Transmit(debug_uart_port.huart, log_msg.buf, log_msg.len, HAL_MAX_DELAY);
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

