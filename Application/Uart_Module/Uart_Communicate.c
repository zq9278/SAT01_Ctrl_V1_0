
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
void UartFrame_Dispatch(FrameId_t frame_id, const uint8_t *data_ptr, uint16_t data_len)
{
    switch (frame_id)
    {
        // =========================================================================
        // ① 压力设定（float, 单位 kPa, 范围 5~39）
        // =========================================================================
        case F32_PRESSURE_SET_KPA:
        {
            float press_kpa = handle_float_data(data_ptr, data_len);
            if (press_kpa < 5.0f)  press_kpa = 5.0f;
            if (press_kpa > 39.0f) press_kpa = 39.0f;
            PressureSet = (u16)(press_kpa * 1000.0f);   // 转换为 Pa 存储
            LOG("[UART] 压力设定 %.2f kPa\r\n", press_kpa);
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
            LOG("[UART] 左眼温度设定 %.2f°C\r\n", lt);
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
            LOG("[UART] 右眼温度设定 %.2f°C\r\n", rt);
            break;
        }

        // =========================================================================
        // ④ 左眼压力开关（uint8_t）
        // =========================================================================
        case U8_LEFT_PRESSURE_ENABLE:
        {
            uint8_t en = handle_uint8_t_data(data_ptr, data_len);
            if (en)
            {
                WorkMode |= 0x01; // 左气压 ON
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = AirPump1PWM;
                LOG("[UART] 左气压 开启\r\n");
               
					AirValve2(1);
					AirValve1(1);
            }
            else
            {
                WorkMode &= ~0x01; // 左气压 OFF
                LOG("[UART] 左气压 关闭\r\n");

                if ((WorkMode & 0x03) == 0)
                {
                    WorkModeState = 0;
                    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
                    TIM15->CCR1 = 0;
                    AirValve1(0);
                    AirValve2(0);
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
            if (en)
            {
                WorkMode |= 0x02; // 右气压 ON
                HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
                TIM15->CCR1 = AirPump1PWM;
                LOG("[UART] 右气压 开启\r\n");
                AirValve1(1);
                AirValve2(1);
            }
            else
            {
                WorkMode &= ~0x02;
                LOG("[UART] 右气压 关闭\r\n");

                if ((WorkMode & 0x03) == 0)
                {
                    WorkModeState = 0;
                    HAL_TIM_PWM_Stop(&htim15, TIM_CHANNEL_1);
                    TIM15->CCR1 = 0;
                    AirValve1(0);
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
                LOG("[UART] 左加热 开启\r\n");
            }
            else
            {
                WorkMode &= ~0x04;  // 左加热 OFF
                HeatPower(2, 0);
                LOG("[UART] 左加热 关闭\r\n");
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
                LOG("[UART] 右加热 开启\r\n");
            }
            else
            {
                WorkMode &= ~0x08;  // 右加热 OFF
                HeatPower(1, 0);
                LOG("[UART] 右加热 关闭\r\n");
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
            LOG("[UART] 气泵功率设定 PWM=%u\r\n", pwm);
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
        //             LOG("[UART] 加水 启动\r\n");
        //         }
        //         else if (p[1] == 1)
        //         {
        //             WorkMode |= 0x10; // 排水
        //             LOG("[UART] 排水 启动\r\n");
        //         }
        //         else if (p[2] == 1)
        //         {
        //             WaterState = StartWaterPump(); // 水泵启
        //             LOG("[UART] 水泵 启动\r\n");
        //         }
        //     }
        //     else if (p[2] == 0)
        //     {
        //         WaterState = 0;
        //         StopWaterPump(); // 停止水泵
        //         LOG("[UART] 水泵 停止\r\n");
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
        //         LOG("[UART] 冷却模块 停止\r\n");
        //     }
        //     else
        //     {
        //         uint8_t power = (data_len > 3) ? data_ptr[3] : 128;
        //         StartIPLCold(power);
        //         LOG("[UART] 冷却模块 启动, 功率=%u\r\n", power);
        //     }
        //     break;
        // }

        // =========================================================================
        // 文本帧 / 未知帧
        // =========================================================================
        case FRAME_ID_TEXT:
            LOG("[UART] 文本帧接收 len=%d\r\n", data_len);
            break;

        default:
            LOG("[UART] 未知 frame_id: 0x%04X len=%d\r\n", frame_id, data_len);
            break;
    }
}
