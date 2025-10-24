//
// Created by zq on 2025/7/4.
//

#ifndef ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
#define ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
// typedef enum {
//     // uint8_t 类型
//     U8_LEFT_PRESSURE_ENABLE   = 0x1001,  // 左眼压力开关 0/1
//     U8_RIGHT_PRESSURE_ENABLE  = 0x1002,  // 右眼压力开关 0/1
//     U8_PUMP_VALUE             = 0x1003,  // 气泵数值 0-255

//     // float 类型
//     F32_PRESSURE_SET_KPA      = 0x1004,  // 压力设定值 5k-39kPa
//     F32_LEFT_TEMP_SET_C       = 0x1005,  // 左眼温度设定
//     F32_RIGHT_TEMP_SET_C      = 0x1006,  // 右眼温度设定

//     // 文本/调试（可选）
//     FRAME_ID_TEXT             = 0x1020,
// } FrameId_t;
/**
 * ID 规划规则：
 *   0x1000 ~ 0x10FF : 上位机 → 下位机 控制命令
 *   0x1100 ~ 0x11FF : 下位机 → 上位机 实时反馈
 */

typedef enum
{
    // ============================================================================
    // 上位机 → 下位机（设定指令）
    // ============================================================================
    /** 000 心跳请求（uint8_t, 固定填1） */
    U8_HEARTBEAT_REQ           = 0x1000,
    /** ① 压力设定值（float, 单位 kPa, 范围5~39kPa） */
    F32_PRESSURE_SET_KPA        = 0x1001,
    /** ② 左眼温度设定值（float, 单位°C） */
    F32_LEFT_TEMP_SET_C         = 0x1002,
    /** ③ 右眼温度设定值（float, 单位°C） */
    F32_RIGHT_TEMP_SET_C        = 0x1003,
    /** ④ 左眼压力开关（uint8_t, 0=关闭,1=开启） */
    U8_LEFT_PRESSURE_ENABLE     = 0x1004,
    /** ⑤ 右眼压力开关（uint8_t, 0=关闭,1=开启） */
    U8_RIGHT_PRESSURE_ENABLE    = 0x1005,
    /** ⑥ 左眼加热开关（uint8_t, 0=关闭,1=开启） */
    U8_LEFT_TEMP_ENABLE         = 0x1006,
    /** ⑦ 右眼加热开关（uint8_t, 0=关闭,1=开启） */
    U8_RIGHT_TEMP_ENABLE        = 0x1007,
    /** ⑧ 气泵功率设定（uint8_t, 0~255 对应PWM占空比） */
    U8_PUMP_POWER_VALUE         = 0x1008,
    /** ⑨ 保压时间设定（uint32_t, 单位 ms） */
    U32_HOLD_TIME_MS            = 0x1009,
    /** ⑩ 泄气时间设定（uint32_t, 单位 ms） */
    U32_RELEASE_TIME_MS         = 0x100A,
    /** ⑪ 挤压模式设定（uint8_t，0=正常/1=交替/2=同步 等自定义） */
    U8_SQUEEZE_MODE             = 0x100B,
    /** 调试文本帧（可选） */
    FRAME_ID_TEXT               = 0x10F0,

    // ============================================================================
    // 下位机 → 上位机（实时数据 / 监测反馈）
    // ============================================================================
    /** 000 心跳应答（uint8_t, 固定回1） */
    U8_HEARTBEAT_ACK           = 0x1100,
    /** ① 左眼压力反馈（float, 单位kPa） */
    F32_LEFT_PRESSURE_VALUE     = 0x1101,
    /** ② 右眼压力反馈（float, 单位kPa） */
    F32_RIGHT_PRESSURE_VALUE    = 0x1102,
    /** ③ 左眼温度反馈（float, 单位°C） */
    F32_LEFT_TEMP_VALUE         = 0x1103,
    /** ④ 右眼温度反馈（float, 单位°C） */
    F32_RIGHT_TEMP_VALUE        = 0x1104,
    /** ⑤ 系统运行状态反馈（uint8_t，可选扩展） */
    U8_SYSTEM_STATE             = 0x1105,
    /** ⑥ 设备告警状态反馈（uint8_t，可选扩展） */
    U8_ALARM_STATE              = 0x1106,
    /** ⑦ 左加热模块存在（uint8_t，0=不存在/1=存在） */
    U8_LEFT_HEATER_PRESENT      = 0x1107,
    /** ⑧ 右加热模块存在（uint8_t，0=不存在/1=存在） */
    U8_RIGHT_HEATER_PRESENT     = 0x1108,
    /** ⑨ 左加热模块熔断（uint8_t，0=正常/1=熔断） */
    U8_LEFT_HEATER_FUSE         = 0x1109,
    /** ⑩ 右加热模块熔断（uint8_t，0=正常/1=熔断） */
    U8_RIGHT_HEATER_FUSE        = 0x110A,
} FrameId_t;



//处理串口接收到的结构体数据
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len);
//处理串口接收到的数据
uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len);
//处理串口接收到的数据
float handle_float_data(const uint8_t *data_ptr, uint16_t data_len);

void UartFrame_Dispatch(FrameId_t frame_id,const uint8_t *data_ptr,uint16_t data_len);

#endif //ELECTRICAL_MUSCLE_QUBEMX_UART_COMMUNICATE_H
