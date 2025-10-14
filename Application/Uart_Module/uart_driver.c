#include "uart_driver.h"
#include <string.h>
#include "Uart_Communicate.h"
#include "user_uart_hal.h"

/**
 * @brief 初始化 UART 驱动
 * @param port     UART 抽象端口（包含环形缓冲、回调函数等）
 * @param instance 硬件 UART 句柄（比如 &huart1）
 * @param baudrate 波特率
 */
void Uart_Init(UartPort_t *port, void *instance, uint32_t baudrate)
{
    port->instance   = instance;
    port->rx_head    = port->rx_tail = 0;   // 接收环形缓冲初始化为空
    port->tx_head    = port->tx_tail = 0;   // 发送环形缓冲初始化为空
    port->rx_callback = rx_callback;        // 指定接收完成后的回调函数

    // 如果需要初始化硬件，可以在这里调用 UART_HAL_Init
    //UART_HAL_Init(instance, baudrate);

    // 使能接收中断 (RXNE)
    UART_HAL_EnableRxInterrupt(instance);

    // 开启 NVIC 中断
    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

/**
 * @brief 发送一个字节
 */
void Uart_SendByte(UartPort_t *port, uint8_t byte)
{
    UART_HAL_SendByte(port->instance, byte);
}

/**
 * @brief 发送一个缓冲区（阻塞方式）
 */
void Uart_SendBuffer(UartPort_t *port, const uint8_t *buf, uint16_t len)
{
    for (uint16_t i = 0; i < len; i++) {
        UART_HAL_SendByte(port->instance, buf[i]);
    }
}

/**
 * @brief 从接收环形缓冲中取一个字节
 * @return true 取到数据 / false 没有数据
 */
bool Uart_ReadByte(UartPort_t *port, uint8_t *out)
{
    if (port->rx_head == port->rx_tail) return false; // 缓冲为空
    *out = port->rx_buf[port->rx_tail];
    port->rx_tail = (port->rx_tail + 1) % UART_RX_BUF_SIZE;
    return true;
}

/**
 * @brief 中断服务函数收到 1 个字节时调用
 * @param port UART 端口
 * @param byte 收到的字节
 */
void UART_Driver_OnRxByte(UartPort_t *port, uint8_t byte)
{
    // 放入环形缓冲
    uint16_t next = (port->rx_head + 1) % UART_RX_BUF_SIZE;
    if (next != port->rx_tail) {
        port->rx_buf[port->rx_head] = byte;
        port->rx_head = next;
    }

    // 立即调用应用层回调函数（逐字节协议解析）
    if (port->rx_callback) {
        port->rx_callback(byte);
    }
}

// UART1 的全局端口对象
extern UartPort_t uart1_port;

/**
 * @brief 接收回调函数，每收到一个字节就会调用
 * @param data 收到的单个字节
 *
 * 处理流程：
 * 1. 每个字节进入 Protocol_ParseByte 状态机。
 * 2. 如果状态机判断收到一帧完整数据，就进入 if 分支。
 * 3. 根据帧的类型和 ID 做业务处理，这里是：
 *    - 如果是 float 类型且 ID == FLOAT_TEMPERATURE：
 *        * 提取 float 数值
 *        * 点亮 / 翻转 LED
 *        * 发送一帧固定的温度 36.5f 回出去
 */
void rx_callback(uint8_t data)
{
    static Frame_t f;

    // 逐字节解析协议
    if (Protocol_ParseByte(data, &f)) {
        // ? 收到完整一帧
        if (f.data_type == DATA_FLOAT && f.frame_id == FLOAT_TEMPERATURE) {
            float val;
            memcpy(&val, f.data, sizeof(float));

            // 业务处理：点亮 LED（低电平有效，这里用翻转）
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
            HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_1);

            // 回发一帧固定的温度数据 36.5f
            float temp = 36.5f;
            Uart_SendFrame(&uart1_port, DATA_FLOAT, FLOAT_TEMPERATURE, &temp);
        }
    }
}