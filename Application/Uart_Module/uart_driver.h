#ifndef UART_DRIVER_H
#define UART_DRIVER_H   
#include <stdbool.h>
#include "FreeRTOS.h"
#include "semphr.h"
#include "stm32g0xx_hal.h"
#include "LOG.h"
#define UART_TX_QUEUE_LENGTH 8
#define UART_TX_MSG_MAX_LEN  50
#define UART_RX_DMA_BUFFER_SIZE 100
#define UART_RX_QUEUE_SIZE 10

#define FRAME_HEADER_1 0xAA
#define FRAME_HEADER_2 0x55
#define FRAME_TAIL_1   0x0D
#define FRAME_TAIL_2   0x0A

#define FRAME_MAX_DATA_LEN 32
#define ASCII_CMD_MAX_LEN 128

typedef enum {
    DATA_TYPE_NONE   = 0x00,
    DATA_UINT8_T     = 0x01,  // 单字节：开�?/枚举/0-255 数�?
    DATA_FLOAT       = 0x02,  // 单精度浮点：压力设定/温度设定
    DATA_TYPE_TEXT   = 0x03,  // 文本（调�?/日志�?
} DataType_t;

typedef struct __attribute__((packed)){
    UART_HandleTypeDef *huart;               // 串口句柄
    uint8_t *dma_rx_buf;  // DMA 接收缓冲
    QueueHandle_t rx_queue;                 // 接收消息队列
    QueueHandle_t tx_queue;                 // 发送消�?队列
    const char *name;                       // 串口名字（调试用�?
    void (*parser)(const uint8_t *buf, uint16_t len); // ? 用户�?定义解析函数
    bool (*sender)( DataType_t type, uint16_t frame_id, const void *data); // ? 用户�?定义发送函数（例�?�打包帧发送）
    uint16_t (*crc)(const uint8_t *buf, uint16_t len); // ? 用户�?定义发送函数（例�?�打包帧发送）
    SemaphoreHandle_t uartTxDoneSem;
} UartPort_t;
typedef struct __attribute__((packed)){
    uint8_t header[2];      // 0xAA, 0x55
    uint16_t frame_id;       // �?ID
    uint8_t data_type;      // 类型
    uint16_t data_length;   // 数据长度（高字节在前�?
    uint8_t data[FRAME_MAX_DATA_LEN]; // 数据
    uint16_t  checksum;       // 校验
    uint8_t tail[2];        // 0x0D, 0x0A
} __attribute__((packed)) Frame_t;
typedef struct __attribute__((packed)){
    uint8_t data[UART_RX_DMA_BUFFER_SIZE];
    uint16_t length;
} UartRxMessage_t;
typedef struct __attribute__((packed)){
    uint8_t data[UART_TX_MSG_MAX_LEN];
    uint16_t length;
} UartTxMessage_t;
typedef struct __attribute__((packed)){
    char line[ASCII_CMD_MAX_LEN];  // 存储完整一�? ASCII 命令
} AsciiCmdMessage_t;
typedef struct __attribute__((packed)) {
    uint8_t fan;
    uint8_t heat;
    uint8_t mist;
} ConfigData_t;


void rk3576_uart_port_Init(UartPort_t *port);
void debug_uart_port_Init(UartPort_t *port);
void rk3576_uart_port_RxCallback(UartPort_t *port, uint16_t size);
void debug_uart_port_RxCallback(UartPort_t *port, uint16_t size);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size);
bool send_rk3576_uart_port_frame(DataType_t type, uint16_t frame_id, const void *data);
void parse_rk3576_uart_port_stream(const uint8_t *buf, uint16_t len);
void parse_debug_uart_port_stream(const uint8_t *buf, uint16_t len);
uint16_t crc16_modbus(const uint8_t *buf, uint16_t len);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
#endif // UART_DRIVER_H
