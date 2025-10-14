#include "Uart_Communicate.h"
#include "uart_driver.h"
#include <stdbool.h>
#include <string.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

/**
 * @brief 打包并发送一帧数据
 * @param port     UART 端口（封装了实例和缓冲）
 * @param type     数据类型 (DataType_t)
 * @param frame_id 帧 ID (FrameId_t)
 * @param data     数据指针
 *
 * 功能：
 *   根据协议格式，将输入数据打包成完整帧：
 *   Header | FrameID | Type | Length | Data | CRC | Tail
 */
void Uart_SendFrame(UartPort_t* port, DataType_t type, uint16_t frame_id, const void* data)
{
    uint8_t buf[64];
    uint8_t* p = buf;
    uint16_t data_len = 0;
    switch (type)
    {
    case DATA_TYPE_CONFIG: data_len = sizeof(ConfigData_t);
        break;
    case DATA_TYPE_SENSOR_DATA: data_len = sizeof(SensorData_t);
        break;
    case DATA_TYPE_TEXT:
        data_len = strlen((const char*)data);
        if (data_len > FRAME_MAX_DATA_LEN) data_len = FRAME_MAX_DATA_LEN;
        break;
    case DATA_FLOAT: data_len = sizeof(float);
        break; // 支持 float 类型
    default: return; // 未知类型，直接返回
    }
    // 帧头
    *p++ = FRAME_HEADER_1;
    *p++ = FRAME_HEADER_2;
    // Frame ID
    memcpy(p, &frame_id, 2);
    p += 2;
    // 数据类型
    *p++ = (uint8_t)type;
    // 数据长度
    memcpy(p, &data_len, 2);
    p += 2;
    // 数据内容
    memcpy(p, data, data_len);
    p += data_len;
    // CRC 校验（从 FrameID 开始，到 Data 结束）
    uint16_t crc = crc16_modbus(buf + 2, 2 + 1 + 2 + data_len);
    memcpy(p, &crc, 2);
    p += 2;
    // 帧尾
    *p++ = FRAME_TAIL_1;
    *p++ = FRAME_TAIL_2;
    // 通过底层驱动逐字节发出
    Uart_SendBuffer(port, buf, p - buf);
}
/**
 * @brief 逐字节协议解析状态机
 * @param byte      新收到的字节
 * @param out_frame 解析成功时输出完整帧
 * @return true  收到完整且 CRC 正确的帧
 *         false 尚未收完或 CRC 错误
 */
bool Protocol_ParseByte(uint8_t byte, Frame_t* out_frame)
{
    // 状态机的几个阶段
    static enum
    {
        STATE_IDLE, // 等待帧头 0xAA
        STATE_HEADER2, // 等待帧头 0x55
        STATE_FIXED, // 接收 FrameID + Type + Len
        STATE_DATA, // 接收 Data[n]
        STATE_CRC, // 接收 CRC
        STATE_TAIL // 接收帧尾
    } state = STATE_IDLE;

    static Frame_t frame; // 临时存放正在组装的帧
    static uint16_t index; // 累计字节计数
    static uint16_t expected_len; // 期望的数据长度
    switch (state)
    {
    case STATE_IDLE:
        if (byte == FRAME_HEADER_1)
        {
            frame.header[0] = byte;
            state = STATE_HEADER2;
        }
        break;
    case STATE_HEADER2:
        if (byte == FRAME_HEADER_2)
        {
            frame.header[1] = byte;
            index = 0;
            state = STATE_FIXED;
        }
        else
        {
            state = STATE_IDLE; // 帧头错误，重置
        }
        break;

    case STATE_FIXED:
        // 收集 FrameID(2) + Type(1) + Len(2)
        ((uint8_t*)&frame.frame_id)[index++] = byte;
        if (index == sizeof(frame.frame_id) + sizeof(frame.data_type) + sizeof(frame.data_length))
        {
            expected_len = frame.data_length;
            if (expected_len > FRAME_MAX_DATA_LEN)
            {
                // 长度异常，丢弃
                state = STATE_IDLE;
            }
            else
            {
                index = 0;
                state = STATE_DATA;
            }
        }
        break;
    case STATE_DATA:
        // 收集数据
        frame.data[index++] = byte;
        if (index >= expected_len)
        {
            index = 0;
            state = STATE_CRC;
        }
        break;
    case STATE_CRC:
        // 收集 CRC 两字节
        ((uint8_t*)&frame.checksum)[index++] = byte;
        if (index == 2)
        {
            index = 0;
            state = STATE_TAIL;
        }
        break;
    case STATE_TAIL:
        // 收集帧尾两个字节
        frame.tail[index++] = byte;
        if (index == 2)
        {
            index = 0;
            state = STATE_IDLE;
            // 校验 CRC + 帧尾
            uint16_t crc_calc = crc16_modbus((uint8_t*)&frame.frame_id, 2 + 1 + 2 + frame.data_length);
            if (crc_calc == frame.checksum &&
                frame.tail[0] == FRAME_TAIL_1 && frame.tail[1] == FRAME_TAIL_2)
            {
                *out_frame = frame; // 输出完整帧
                return true;
            }
        }
        break;
    }
    return false;
}

/**
 * @brief 计算 Modbus CRC16
 * @param buf 输入数据
 * @param len 数据长度
 * @return 16 位 CRC
 */
uint16_t crc16_modbus(const uint8_t* buf, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= buf[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x0001)
                crc = (crc >> 1) ^ 0xA001;
            else
                crc >>= 1;
        }
    }
    return crc;
}