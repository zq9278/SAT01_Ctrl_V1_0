
// Created by zq on 2025/7/4.
//
#include <ctype.h>
#include <string.h>
#include "Uart_Communicate.h"
#include <stdbool.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "semphr.h"
#include "stm32g0xx_hal.h"
#include "uart_driver.h"
#include "LOG.h"
// control modules
#include "apply.h"
#include "heat.h"
#include "tim.h"

// externs from application domain
extern volatile u8 WorkMode;
extern volatile u8 WorkModeState;
extern volatile u8 AirPump1PWM;
extern volatile u16 PressureSet;
extern volatile u16 RightTempSet, LeftTempSet;
extern PID_TypeDef RightHeat, LeftHeat;
extern uint32_t holdSeconds ;   // 保压时间（m秒）
extern uint32_t releaseSeconds;// 泄气时间（m秒）
extern TimerHandle_t xHoldTimer;

// New protocol states
static volatile uint8_t s_squeeze_mode = 0; // 0=normal, 1=alternate, 2=sync (user-defined)
extern UartPort_t rk3576_uart_port;

//澶勭悊涓插彛鎺ユ敹鍒扮殑缁撴瀯浣撴暟鎹?
void handle_config_data(const uint8_t* data_ptr, uint16_t data_len)
{
    // // 鏍￠獙闀垮害
    // if(data_len < sizeof(ConfigData_t)) {
    //     LOG("[Wave] DATA_TYPE_CONFIG 鏁版嵁闀垮害閿欒??: %d\n", data_len);
    //     return;
    // }

    // ConfigData_t s;
    // memcpy(&s, data_ptr, sizeof(ConfigData_t));

    // pm_cmd_t cmd = { .id = PM_CMD_SET_FAN_LEVEL };
    // cmd.u.fan_level = 100-s.fan;
    // Param_Post(&cmd, pdMS_TO_TICKS(10));

    // pm_cmd_t cmd2 = { .id = PM_CMD_SET_MIST_LEVEL };
    // cmd.u.mist_level = s.mist;
    // Param_Post(&cmd2, pdMS_TO_TICKS(10));

}
//澶勭悊涓插彛鎺ユ敹鍒扮殑鏁版嵁
uint8_t handle_uint8_t_data(const uint8_t *data_ptr, uint16_t data_len)
{
    // 鏍￠獙闀垮害
    if(data_len < sizeof(uint8_t)) {
        LOG("[Wave] DATA_uint8_t 鏁版嵁闀垮害閿欒??: %d\n", data_len);
        return 0;
    }
    uint8_t s;
    memcpy(&s, data_ptr, sizeof(uint8_t));
 return s;
}
//澶勭悊涓插彛鎺ユ敹鍒扮殑鏁版嵁
float handle_float_data(const uint8_t *data_ptr, uint16_t data_len)
{
    // 鏍￠獙闀垮害
    if(data_len < sizeof(float)) {
        LOG("[Wave] DATA_uint8_t 鏁版嵁闀垮害閿欒??: %d\n", data_len);
        return 0;
    }
    float s;
    memcpy(&s, data_ptr, sizeof(float));
    return s;
}
/**
 * @brief  UART帧调度函数
 * @param  frame_id  帧ID（参考 FrameId_t 枚举）
 * @param  data_ptr  数据指针
 * @param  data_len  数据长度
 *
 * @note   本函数由上位机命令触发，修改全局控制变量或 WorkMode 位字段。
 *         WorkMode 位定义：
 *           bit0 → 左气压功能开启
 *           bit1 → 右气压功能开启
 *           bit2 → 左加热功能开启
 *           bit3 → 右加热功能开启
 */
 extern WorkState_t workState;
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        // =========================================================================
        // 000 心跳请求（uint8_t） -> 立即回 0x1100 心跳应答
        // =========================================================================
        case U8_HEARTBEAT_REQ:
        {
            uint8_t hb = 1;
            (void)handle_uint8_t_data(data_ptr, data_len); // 允许上位机传值但目前不使用
            rk3576_uart_port.sender(DATA_UINT8_T, U8_HEARTBEAT_ACK, &hb);
            LOG("  心跳请求 -> 应答\r\n");
            break;
        }
        // =========================================================================
        // ① 压力设定（float, 单位 kPa, 范围 5~39）
        // =========================================================================
        case F32_PRESSURE_SET_KPA:
        {
            float press_kpa = handle_float_data(data_ptr, data_len);
            if (press_kpa < 5.0f)  press_kpa = 5.0f;
            if (press_kpa > 39.0f) press_kpa = 39.0f;
            PressureSet = (u16)(press_kpa * 1000.0f);   // 转换为 Pa 存储
            LOG("  压力设定 %.2f kPa\r\n", press_kpa);
            break;
        }

        // =========================================================================
        // ② 左眼温度设定（float, °C）
        // =========================================================================
        case F32_LEFT_TEMP_SET_C:
        {
            float lt = handle_float_data(data_ptr, data_len);
            int32_t sp = (int32_t)(lt * 100.0f);
            LeftTempSet = (u16)sp;

            PID_Init(&LeftHeat, 400, 2, 200, 100000, 0, 1999, 0, LeftTempSet);
            LOG("  左眼温度设定 %.2f°C\r\n", lt);
            break;
        }

        // =========================================================================
        // ③ 右眼温度设定（float, °C）
        // =========================================================================
        case F32_RIGHT_TEMP_SET_C:
        {
            float rt = handle_float_data(data_ptr, data_len);
            int32_t sp = (int32_t)(rt * 100.0f);
            RightTempSet = (u16)sp;

            PID_Init(&RightHeat, 400, 2, 200, 100000, 0, 1999, 0, RightTempSet);
            LOG("  右眼温度设定 %.2f°C\r\n", rt);
            break;
        }
        // =========================================================================
        // ⑩ 泄气时间设定（uint32_t, 单位 ms）
        // =========================================================================
        case U32_RELEASE_TIME_MS:
        {
            if (data_len >= sizeof(uint32_t))
            {
                uint32_t val_ms;
                memcpy(&val_ms, data_ptr, sizeof(uint32_t));
                releaseSeconds = val_ms;
                LOG("  泄气时间设定 %lu ms\r\n", releaseSeconds);
            }
            else
            {
                LOG("  泄气时间数据长度错误 len=%d\r\n", data_len);
            }
            break;
        }
        case U32_HOLD_TIME_MS:
        {
            if (data_len >= sizeof(uint32_t))
            {
                uint32_t val_ms;
                memcpy(&val_ms, data_ptr, sizeof(uint32_t));
                holdSeconds = val_ms;
                LOG("  保压时间设定 %lu ms\r\n", holdSeconds);
            
                // ✅ 如果定时器已经创建，立刻应用新的时间
                if (xHoldTimer != NULL)
                {
                    // 改变周期并重启定时器
                    xTimerChangePeriod(xHoldTimer, pdMS_TO_TICKS(holdSeconds), 0);
                }
            }
            break;
        }

        // =========================================================================
        // ④ 左眼压力开关（uint8_t）
        // =========================================================================
        case U8_LEFT_PRESSURE_ENABLE:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
             workState = WORK_IDLE;
            if (en)
            {
                //workState = WORK_IDLE;
                WorkMode |= 0x01; // 左气压 ON
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = AirPump1PWM;
                LOG("  左气压 开启\r\n");
            }
            else
            {
                WorkMode &= ~0x01; // 左气压 OFF
                LOG("  左气压 关闭\r\n");
                 AirValve2(0);
                if ((WorkMode & 0x03) == 0)
                {
                    //WorkModeState = 0;
                    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
                    TIM15->CCR1 = 0;
                     AirValve1(0);
					
                }
            }
            break;
        }

        // =========================================================================
        // ⑤ 右眼压力开关（uint8_t）
        // =========================================================================
        case U8_RIGHT_PRESSURE_ENABLE:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
             workState = WORK_IDLE;
            if (en)
            {
                WorkMode |= 0x02; // 右气压 ON
               
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = AirPump1PWM;
                LOG("  右气压 开启\r\n");
            }
            else
            {
                WorkMode &= ~0x02;
                LOG("  右气压 关闭\r\n");
                AirValve1(0);
                if ((WorkMode & 0x03) == 0)
                {
                    // WorkModeState = 0;
                    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
                    TIM15->CCR1 = 0;
					AirValve2(0);
                }
            }
            break;
        }

        // =========================================================================
        // ⑥ 左眼加热开关（uint8_t）
        // =========================================================================
        case U8_LEFT_TEMP_ENABLE:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
            if (en)
            {
                WorkMode |= 0x04;   // 左加热 ON
                HeatPower(2, 1);
                HeatPWMSet(2, 0);
                LOG("  左加热 开启\r\n");
            }
            else
            {
                WorkMode &= ~0x04;  // 左加热 OFF
                HeatPower(2, 0);
                LOG("  左加热 关闭\r\n");
            }
            break;
        }

        // =========================================================================
        // ⑦ 右眼加热开关（uint8_t）
        // =========================================================================
        case U8_RIGHT_TEMP_ENABLE:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
            if (en)
            {
                WorkMode |= 0x08;   // 右加热 ON
                HeatPower(1, 1);
                HeatPWMSet(1, 0);
                LOG("  右加热 开启\r\n");
            }
            else
            {
                WorkMode &= ~0x08;  // 右加热 OFF
                HeatPower(1, 0);
                LOG("  右加热 关闭\r\n");
            }
            break;
        }

        // =========================================================================
        // ⑧ 气泵功率设定（uint8_t, 0~255）
        // =========================================================================
        case U8_PUMP_POWER_VALUE:
        {
            uint8_t pwm = handle_uint8_t_data(data_ptr, data_len);
            AirPump1PWM = pwm;

            if ((WorkMode & 0x03) != 0)
            {
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = AirPump1PWM;
            }
            LOG("  气泵功率设定 PWM=%u\r\n", pwm);
            break;
        }

        // =========================================================================
        // ⑪ 挤压模式设定（uint8_t）
        // =========================================================================
        case U8_SQUEEZE_MODE:
        {
            s_squeeze_mode = handle_uint8_t_data(data_ptr, data_len);
            LOG("  挤压模式设定=%u\r\n", s_squeeze_mode);
            break;
        }

        // // =========================================================================
        // // ⑨ 水路控制命令（扩展：加水/排水/水泵启停）
        // // =========================================================================
        // case 0x1010: // U8_WATER_CTRL（扩展保留）
        // {
        //     const uint8_t *p = data_ptr;
        //     if (p[0] + p[1] + p[2] == 1 && (WorkMode & 0xF0) == 0)
        //     {
        //         if (p[0] == 1)
        //         {
        //             WorkMode |= 0x20; // 加水
        //             LOG("  加水 启动\r\n");
        //         }
        //         else if (p[1] == 1)
        //         {
        //             WorkMode |= 0x10; // 排水
        //             LOG("  排水 启动\r\n");
        //         }
        //         else if (p[2] == 1)
        //         {
        //             WaterState = StartWaterPump(); // 水泵启
        //             LOG("  水泵 启动\r\n");
        //         }
        //     }
        //     else if (p[2] == 0)
        //     {
        //         WaterState = 0;
        //         StopWaterPump(); // 停止水泵
        //         LOG("  水泵 停止\r\n");
        //     }
        //     break;
        // }

        // // =========================================================================
        // // ⑩ 冷却控制命令（扩展）
        // // =========================================================================
        // case 0x1011: // U8_IPL_COLD_ENABLE
        // {
        //     uint8_t en = handle_uint8_t_data(data_ptr, data_len);
        //     if (en == 0x00)
        //     {
        //         StopIPLCold();
        //         LOG("  冷却模块 停止\r\n");
        //     }
        //     else
        //     {
        //         uint8_t power = (data_len > 3) ? data_ptr[3] : 128;
        //         StartIPLCold(power);
        //         LOG("  冷却模块 启动, 功率=%u\r\n", power);
        //     }
        //     break;
        // }

        // =========================================================================
        // 文本帧 / 未知帧
        // =========================================================================
        case FRAME_ID_TEXT:
            LOG("  文本帧接收 len=%d\r\n", data_len);
            break;

        default:
            LOG("  未知 frame_id: 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}
