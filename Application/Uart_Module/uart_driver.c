#include "uart_driver.h"
#include <string.h>
#include "Uart_Communicate.h"
#include "user_uart_hal.h"

/**
 * @brief ��ʼ�� UART ����
 * @param port     UART ����˿ڣ��������λ��塢�ص������ȣ�
 * @param instance Ӳ�� UART ��������� &huart1��
 * @param baudrate ������
 */
void Uart_Init(UartPort_t *port, void *instance, uint32_t baudrate)
{
    port->instance   = instance;
    port->rx_head    = port->rx_tail = 0;   // ���ջ��λ����ʼ��Ϊ��
    port->tx_head    = port->tx_tail = 0;   // ���ͻ��λ����ʼ��Ϊ��
    port->rx_callback = rx_callback;        // ָ��������ɺ�Ļص�����

    // �����Ҫ��ʼ��Ӳ����������������� UART_HAL_Init
    //UART_HAL_Init(instance, baudrate);

    // ʹ�ܽ����ж� (RXNE)
    UART_HAL_EnableRxInterrupt(instance);

    // ���� NVIC �ж�
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief ����һ���ֽ�
 */
void Uart_SendByte(UartPort_t *port, uint8_t byte)
{
    UART_HAL_SendByte(port->instance, byte);
}

/**
 * @brief ����һ����������������ʽ��
 */
void Uart_SendBuffer(UartPort_t *port, const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_HAL_SendByte(port->instance, buf[i]);
    }
}

/**
 * @brief �ӽ��ջ��λ�����ȡһ���ֽ�
 * @return true ȡ������ / false û������
 */
bool Uart_ReadByte(UartPort_t *port, uint8_t *out)
{
    if (port->rx_head == port->rx_tail) return false; // ����Ϊ��
    *out = port->rx_buf[port->rx_tail];
    port->rx_tail = (port->rx_tail + 1) % UART_RX_BUF_SIZE;
    return true;
}

/**
 * @brief �жϷ������յ� 1 ���ֽ�ʱ����
 * @param port UART �˿�
 * @param byte �յ����ֽ�
 */
void UART_Driver_OnRxByte(UartPort_t *port, uint8_t byte)
{
    // ���뻷�λ���
    uint16_t next = (port->rx_head + 1) % UART_RX_BUF_SIZE;
    if (next != port->rx_tail) {
        port->rx_buf[port->rx_head] = byte;
        port->rx_head = next;
    }

    // ��������Ӧ�ò�ص����������ֽ�Э�������
    if (port->rx_callback) {
        port->rx_callback(byte);
    }
}

// UART1 ��ȫ�ֶ˿ڶ���
extern UartPort_t uart1_port;

/**
 * @brief ���ջص�������ÿ�յ�һ���ֽھͻ����
 * @param data �յ��ĵ����ֽ�
 *
 * �������̣�
 * 1. ÿ���ֽڽ��� Protocol_ParseByte ״̬����
 * 2. ���״̬���ж��յ�һ֡�������ݣ��ͽ��� if ��֧��
 * 3. ����֡�����ͺ� ID ��ҵ���������ǣ�
 *    - ����� float ������ ID == FLOAT_TEMPERATURE��
 *        * ��ȡ float ��ֵ
 *        * ���� / ��ת LED
 *        * ����һ֡�̶����¶� 36.5f �س�ȥ
 */
void rx_callback(uint8_t data)
{
    static Frame_t f;

    // ���ֽڽ���Э��
    if (Protocol_ParseByte(data, &f)) {
        // ? �յ�����һ֡
        if (f.data_type == DATA_FLOAT && f.frame_id == FLOAT_TEMPERATURE) {
            float val;
            memcpy(&val, f.data, sizeof(float));

            // ҵ�������� LED���͵�ƽ��Ч�������÷�ת��
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

            // �ط�һ֡�̶����¶����� 36.5f
            float temp = 36.5f;
            Uart_SendFrame(&uart1_port, DATA_FLOAT, FLOAT_TEMPERATURE, &temp);
        }
    }
}