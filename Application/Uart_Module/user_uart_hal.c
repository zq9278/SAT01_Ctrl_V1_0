#include "user_uart_hal.h"
#include "main.h"
#include "uart_driver.h"
#include "stm32f0xx_hal.h"  // ����оƬ�ͺ��滻

extern UART_HandleTypeDef huart1;  // CubeMX ����
extern UartPort_t uart1_port;      // �� main.c ����

/* ��ʼ��Ӳ�� UART */
void UART_HAL_Init(void *instance, uint32_t baudrate)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef*)instance;
    huart->Init.BaudRate = baudrate;
    huart->Init.WordLength = UART_WORDLENGTH_8B;
    huart->Init.StopBits = UART_STOPBITS_1;
    huart->Init.Parity = UART_PARITY_NONE;
    huart->Init.Mode = UART_MODE_TX_RX;
    huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart->Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(huart) != HAL_OK) {
        Error_Handler();
    }
}

/* ����һ���ֽڣ��������ɣ� */
void UART_HAL_SendByte(void *instance, uint8_t byte)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef*)instance;
    while (!(huart->Instance->ISR & USART_ISR_TXE));  // �ȴ�TXE
    huart->Instance->TDR = byte;
}

/* ���������ж� */
void UART_HAL_EnableRxInterrupt(void *instance)
{
    UART_HandleTypeDef *huart = (UART_HandleTypeDef*)instance;
    __HAL_UART_ENABLE_IT(huart, UART_IT_RXNE);   // �� RXNE �ж�
    __HAL_UART_ENABLE_IT(huart, UART_IT_ERR);    // ����ͬʱ�������ж�
}

/* �����жϷ����� (�� stm32f0xx_it.c) */
void USART1_IRQHandler(void)
{
    // RXNE�����ջ���ǿ�
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE)) {
        uint8_t data = (uint8_t)(huart1.Instance->RDR & 0xFF);
        UART_Driver_OnRxByte(&uart1_port, data);
    }

    // ��������־ (��ѡ)
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_ORE)) {
        __HAL_UART_CLEAR_OREFLAG(&huart1);
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_FE)) {
        __HAL_UART_CLEAR_FEFLAG(&huart1);
    }
    if (__HAL_UART_GET_FLAG(&huart1, UART_FLAG_NE)) {
        __HAL_UART_CLEAR_NEFLAG(&huart1);
    }
}