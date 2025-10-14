//
// Created by zq on 2025/7/4.
//

#ifndef ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
#define ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
#include "stm32f0xx_hal.h"
#include "uart_driver.h"

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
    CMD_UNKNOWN = 0,
    CMD_SET,
    CMD_GET,
    CMD_RESET
} CommandType_t;
typedef struct __attribute__((packed)){
    char line[ASCII_CMD_MAX_LEN];  // 存储完整一行 ASCII 命令
} AsciiCmdMessage_t;
typedef struct __attribute__((packed)){
    char cmd[8];         // "set", "get", "reset"
    char device[16];     // "dac1" ~ "dac4"
    char param[16];      // "freq", "amp", "type"
    float value;         // 支持浮点参数
    char *raw_line;  // ? 保存原始指令行
} AsciiCommand_t;

// 主解析函数
bool parse_ascii_command(const char *line, AsciiCommand_t *out_cmd);

// 用户调用该函数来执行解析后逻辑
void handle_ascii_command(const AsciiCommand_t *cmd);

typedef struct __attribute__((packed)){
    uint8_t data[UART_RX_DMA_BUFFER_SIZE];
    uint16_t length;
} UartRxMessage_t;

typedef struct __attribute__((packed)){
    uint8_t data[UART_TX_MSG_MAX_LEN];
    uint16_t length;
} UartTxMessage_t;

typedef struct __attribute__((packed)){
    uint8_t header[2];      // 0xAA, 0x55
    uint16_t frame_id;       // 帧ID
    uint8_t data_type;      // 类型
    uint16_t data_length;   // 数据长度（高字节在前）
    uint8_t data[FRAME_MAX_DATA_LEN]; // 数据
    uint16_t  checksum;       // 校验
    uint8_t tail[2];        // 0x0D, 0x0A
} __attribute__((packed)) Frame_t;

typedef enum {
    DATA_TYPE_NONE         = 0x00,
    DATA_TYPE_CONFIG       = 0x01,  // 配置参数结构体
    DATA_TYPE_SENSOR_DATA  = 0x02,  // 传感器数据结构体
    DATA_TYPE_COMMAND      = 0x03,  // 控制命令结构体
    DATA_TYPE_RESPONSE     = 0x04,  // 应答信息
    DATA_TYPE_TEXT         = 0x05,   // 文本信息
    DATA_CHANNEL_VALUE          = 0x06,   // 通道详细信息
    DATA_UINT8_T          = 0x07,   // 通道详细信息
    DATA_UINT16_T          = 0x08,   // 通道详细信息
    DATA_FLOAT          = 0x09,   // 通道详细信息
    // 可以扩展更多
} DataType_t;
typedef enum {
    FRAME_ID_CONFIG   = 0x1000,
    UINT8_T_FAN      = 0x1001,
    UINT8_T_MIST     = 0x1002,
    FLOAT_TEMPERATURE   = 0x1003,
    UINT8_T_MODEL     = 0x1004,
    UINT8_T_RUNSTATE    = 0x1005,
    UINT8_T_WATER_LOW     = 0x1006,
    FRAME_ID_SENSOR   = 0x1010,
    FRAME_ID_TEXT     = 0x1020,
} FrameId_t;

typedef struct __attribute__((packed)) {
    uint8_t fan;
    uint8_t heat;
    uint8_t mist;
} ConfigData_t;

typedef struct __attribute__((packed)) {
    float temperature;
} ChannelValue_t;

typedef struct __attribute__((packed)){
    uint32_t adc_data;
} SensorData_t;

typedef struct __attribute__((packed)){
    uint8_t command_id;
    uint8_t param;
} CommandData_t;

typedef struct __attribute__((packed)){
    uint8_t result;
    char message[32];
} ResponseData_t;


typedef union {
    ConfigData_t     config;
    SensorData_t     sensor;
    CommandData_t    command;
    ResponseData_t   response;
    ChannelValue_t   channel_value;
    char             text[FRAME_MAX_DATA_LEN];  // 最后兜底
} PayloadUnion_t;

void Uart_SendFrame(UartPort_t *port, DataType_t type, uint16_t frame_id, const void *data);
bool Protocol_ParseByte(uint8_t byte, Frame_t *out_frame);
uint16_t crc16_modbus(const uint8_t *buf, uint16_t len);
#endif //ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H