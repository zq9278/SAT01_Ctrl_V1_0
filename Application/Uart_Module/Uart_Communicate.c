#include "Uart_Communicate.h"
#include "uart_driver.h"
#include <stdbool.h>
#include <string.h>
#include "main.h"

extern UART_HandleTypeDef huart1;

/**
 * @brief ���������һ֡����
 * @param port     UART �˿ڣ���װ��ʵ���ͻ��壩
 * @param type     �������� (DataType_t)
 * @param frame_id ֡ ID (FrameId_t)
 * @param data     ����ָ��
 *
 * ���ܣ�
 *   ����Э���ʽ�����������ݴ��������֡��
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
        break; // ֧�� float ����
    default: return; // δ֪���ͣ�ֱ�ӷ���
    }
    // ֡ͷ
    *p++ = FRAME_HEADER_1;
    *p++ = FRAME_HEADER_2;
    // Frame ID
    memcpy(p, &frame_id, 2);
    p += 2;
    // ��������
    *p++ = (uint8_t)type;
    // ���ݳ���
    memcpy(p, &data_len, 2);
    p += 2;
    // ��������
    memcpy(p, data, data_len);
    p += data_len;
    // CRC У�飨�� FrameID ��ʼ���� Data ������
    uint16_t crc = crc16_modbus(buf + 2, 2 + 1 + 2 + data_len);
    memcpy(p, &crc, 2);
    p += 2;
    // ֡β
    *p++ = FRAME_TAIL_1;
    *p++ = FRAME_TAIL_2;
    // ͨ���ײ��������ֽڷ���
    Uart_SendBuffer(port, buf, p - buf);
}
/**
 * @brief ���ֽ�Э�����״̬��
 * @param byte      ���յ����ֽ�
 * @param out_frame �����ɹ�ʱ�������֡
 * @return true  �յ������� CRC ��ȷ��֡
 *         false ��δ����� CRC ����
 */
bool Protocol_ParseByte(uint8_t byte, Frame_t* out_frame)
{
    // ״̬���ļ����׶�
    static enum
    {
        STATE_IDLE, // �ȴ�֡ͷ 0xAA
        STATE_HEADER2, // �ȴ�֡ͷ 0x55
        STATE_FIXED, // ���� FrameID + Type + Len
        STATE_DATA, // ���� Data[n]
        STATE_CRC, // ���� CRC
        STATE_TAIL // ����֡β
    } state = STATE_IDLE;

    static Frame_t frame; // ��ʱ���������װ��֡
    static uint16_t index; // �ۼ��ֽڼ���
    static uint16_t expected_len; // ���������ݳ���
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
            state = STATE_IDLE; // ֡ͷ��������
        }
        break;

    case STATE_FIXED:
        // �ռ� FrameID(2) + Type(1) + Len(2)
        ((uint8_t*)&frame.frame_id)[index++] = byte;
        if (index == sizeof(frame.frame_id) + sizeof(frame.data_type) + sizeof(frame.data_length))
        {
            expected_len = frame.data_length;
            if (expected_len > FRAME_MAX_DATA_LEN)
            {
                // �����쳣������
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
        // �ռ�����
        frame.data[index++] = byte;
        if (index >= expected_len)
        {
            index = 0;
            state = STATE_CRC;
        }
        break;
    case STATE_CRC:
        // �ռ� CRC ���ֽ�
        ((uint8_t*)&frame.checksum)[index++] = byte;
        if (index == 2)
        {
            index = 0;
            state = STATE_TAIL;
        }
        break;
    case STATE_TAIL:
        // �ռ�֡β�����ֽ�
        frame.tail[index++] = byte;
        if (index == 2)
        {
            index = 0;
            state = STATE_IDLE;
            // У�� CRC + ֡β
            uint16_t crc_calc = crc16_modbus((uint8_t*)&frame.frame_id, 2 + 1 + 2 + frame.data_length);
            if (crc_calc == frame.checksum &&
                frame.tail[0] == FRAME_TAIL_1 && frame.tail[1] == FRAME_TAIL_2)
            {
                *out_frame = frame; // �������֡
                return true;
            }
        }
        break;
    }
    return false;
}

/**
 * @brief ���� Modbus CRC16
 * @param buf ��������
 * @param len ���ݳ���
 * @return 16 λ CRC
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