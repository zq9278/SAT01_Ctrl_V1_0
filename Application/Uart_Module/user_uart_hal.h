#ifndef USER_UART_HAL_H
#define USER_UART_HAL_H

#include <stdint.h>

/* 
 * �û���������ֲʱ�ṩ���º���
 * instance ������ USART1 ����ַ / SDK ��� / �û��Զ���ṹ��
 */

/* ��ʼ��Ӳ�� UART */
void UART_HAL_Init(void *instance, uint32_t baudrate);

/* ����һ���ֽڣ��������ɣ� */
void UART_HAL_SendByte(void *instance, uint8_t byte);

/* ʹ�ܽ����жϣ��յ��ֽ�ʱӦ���� UART_Driver_OnRxByte�� */
void UART_HAL_EnableRxInterrupt(void *instance);

#endif