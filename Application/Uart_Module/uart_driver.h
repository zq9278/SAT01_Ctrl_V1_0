#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

#define UART_RX_BUF_SIZE 128
#define UART_TX_BUF_SIZE 128

typedef struct {
    void *instance;   // Ӳ����ʶ
    uint8_t rx_buf[UART_RX_BUF_SIZE];
    volatile uint16_t rx_head;
    volatile uint16_t rx_tail;

    uint8_t tx_buf[UART_TX_BUF_SIZE];
    volatile uint16_t tx_head;
    volatile uint16_t tx_tail;

    void (*rx_callback)(uint8_t data);  // Ӧ�ò�ص�
} UartPort_t;

/* ��ʼ�� */
void Uart_Init(UartPort_t *port, void *instance, uint32_t baudrate);

/* ���ͽӿ� */
void Uart_SendByte(UartPort_t *port, uint8_t byte);
void Uart_SendBuffer(UartPort_t *port, const uint8_t *buf, uint16_t len);

/* ��ȡ���ջ��� */
bool Uart_ReadByte(UartPort_t *port, uint8_t *out);

/* �жϷ����е��� */
void UART_Driver_OnRxByte(UartPort_t *port, uint8_t byte);
void rx_callback(uint8_t data);
#endif