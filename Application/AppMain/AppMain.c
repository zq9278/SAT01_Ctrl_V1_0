// Created by zq on 2025/5/21.

#include "AppMain.h"
#include "SYS.h"
#include "uart_driver.h"
#include "user_uart_hal.h"
#include "Uart_Communicate.h"
#include "Uart_Task.h"

// FreeRTOS native API
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

// App dependencies used by tasks
#include <string.h>
#include "main.h"
#include "heat.h"
#include "ads1248.h"
#include "Pressure_sensor.h"
#include "apply.h"
#include "ds18b20.h"
#include "water.h"


// ---------------- FreeRTOS native tasks (migrated from Core/Src/app_freertos.c) ----------------

// Globals
volatile u8 OTP1_RESET_Value, OTP2_RESET_Value;
volatile u16 RTDTemperature[2];
volatile s16 WaterTemperature;
volatile u8 WaterSensor;

extern volatile u8 WaterState;
extern volatile u8 WorkMode;
extern PID_TypeDef RightHeat, LeftHeat;

extern volatile RingBuffer_t uart_ring_buffer;
extern volatile uint8_t uart_rx_byte;
extern u16 left_pressure, right_pressure;

extern UartPort_t rk3576_uart_port;
extern UartPort_t debug_uart_port;

// Task prototypes
static void vTaskRTDRead(void *argument);
static void vTaskPressureRead(void *argument);
static void vTaskWorkMode(void *argument);
static void vTaskLowFreqMonitor(void *argument);

void AppMain_FreeRTOS_Init(void)
{
    // Init UART ports (binary protocol + debug)
    rk3576_uart_port_Init(&rk3576_uart_port);
    debug_uart_port_Init(&debug_uart_port);
    BaseType_t ret;
// === UART_RX ===
ret = xTaskCreate(UartRxParserTask, "UART_RX", 256, NULL, 22, NULL);
if (ret != pdPASS)
{
    LOG("[ERR] UART_RX create failed!\r\n");
}
// === UART_TX ===
ret = xTaskCreate(SerialTxTask, "UART_TX", 256, NULL, 21, NULL);
if (ret != pdPASS)
{
    LOG("[ERR] UART_TX create failed!\r\n");
}
// === WorkModeTask ===
ret = xTaskCreate(vTaskWorkMode, "WorkModeTask", 256, NULL, 20, NULL);
if (ret != pdPASS)
{
    LOG("[ERR] WorkModeTask create failed!\r\n");
}
// === PressureRead ===
ret = xTaskCreate(vTaskPressureRead, "PressureRead", 256, NULL, 10, NULL);
if (ret != pdPASS)
{
    LOG("[ERR] PressureRead create failed!\r\n");
}
// === RTDRead ===
ret = xTaskCreate(vTaskRTDRead, "RTDRead", 256, NULL, 2, NULL);
if (ret != pdPASS)
{
    LOG("[ERR] RTDRead create failed!\r\n");
}
// === LowFreq_Task ===
ret = xTaskCreate(vTaskLowFreqMonitor, "LowFreqMon", 256, NULL, 1, NULL);
if (ret != pdPASS)
{
    LOG("[ERR] LowFreq_Task create failed!\r\n");
}
}
static void vTaskLowFreqMonitor(void *argument)
{
    (void)argument;
    uint16_t tick_500ms = 0;

    for (;;)
    {
        // === 500ms 周期 ===
        HAL_GPIO_TogglePin(LED0_GPIO_Port, LED0_Pin);
        HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);

        OTP1_RESET_Value = HAL_GPIO_ReadPin(OTP1_RESET_GPIO_Port, OTP1_RESET_Pin);
        OTP2_RESET_Value = HAL_GPIO_ReadPin(OTP2_RESET_GPIO_Port, OTP2_RESET_Pin);

        tick_500ms++;
        // === 每 5s 周期 ===
        if (tick_500ms >= 10)
        {//低水位检测
            // tick_500ms = 0;
            // WaterTemperature = DS18B20_Get_Temp();
            // WaterSensor = HAL_GPIO_ReadPin(WS1_GPIO_Port, WS1_Pin) ^ 0x01;
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void vTaskRTDRead(void *argument)
{
    (void)argument;
    u8 RTDChannel = RTD1;
    for (;;) {
        if (HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port, RTD_RDY_Pin) == GPIO_PIN_RESET) {
            RTDTemperature[RTDChannel] = ADC2Temperature(ADS1248_Read());
            RTDChannel = 1 ^ RTDChannel;
            ADS1248_ChangeChannel(RTDChannel);
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
}

static void vTaskPressureRead(void *argument)
{
    (void)argument;
    for (;;) {
        pressure_sensor_read();
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
/**
 * @brief  主工作模式任务
 *         周期：10ms 调度（通过计数器生成 100ms / 500ms / 5s 周期任务）
 *
 * 功能：
 *   - 根据 WorkMode 各位控制压力/加热功能
 *   - 周期性读取传感器并发送反馈帧
 *   - 与新版 FrameId_t 协议保持一致：
 *       bit0 → 左气压
 *       bit1 → 右气压
 *       bit2 → 左加热
 *       bit3 → 右加热
 */
static void vTaskWorkMode(void *argument)
{
    (void)argument;
    u8 timer_100ms = 0, timer_500ms = 0;
    u16 timer_5000ms = 0;

    for (;;)
    {
        // ===========================================================
        // 周期计数（每10ms一次）
        // ===========================================================
        if (timer_100ms >= 10)  timer_100ms = 0;     // 100ms
        if (timer_500ms >= 50)  timer_500ms = 0;     // 500ms
        if (timer_5000ms >= 500) timer_5000ms = 0;   // 5000ms

        // ===========================================================
        // 【1】周期发送系统运行状态（5s 一次）
        // ===========================================================
        if (timer_5000ms == 0)
        {
            // 发送系统状态（例如水路状态）
            rk3576_uart_port.sender(DATA_UINT8_T, U8_SYSTEM_STATE, &WaterState);

            // 同时发送水温（摄氏度）
            rk3576_uart_port.sender(DATA_FLOAT, F32_LEFT_TEMP_VALUE, &WaterTemperature);
        }

        // ===========================================================
        // 【2】气压控制逻辑 (bit0=左气压, bit1=右气压)
        // ===========================================================
        if ((WorkMode & 0x03) != 0)
        {
            // 执行左右气压表达逻辑
            Work_Expression(WorkMode & 0x03);

            // 每100ms发送左右压力反馈
            if (timer_100ms == 0)
            {
                float left_kpa  = left_pressure / 1000.0f;   // 转kPa
                float right_kpa = right_pressure / 1000.0f;

                rk3576_uart_port.sender(DATA_FLOAT, F32_LEFT_PRESSURE_VALUE,  &left_kpa);
                rk3576_uart_port.sender(DATA_FLOAT, F32_RIGHT_PRESSURE_VALUE, &right_kpa);
            }
        }

        // ===========================================================
        // 【3】加热控制逻辑 (bit2=左加热, bit3=右加热)
        // ===========================================================
        if ((WorkMode & 0x0C) != 0)
        {
            // 执行左右加热逻辑
            Work_Heat((WorkMode & 0x0C) >> 2);

            // 每100ms上报左右眼温度
            if (timer_100ms == 0)
            {
                float  right_temp = (float)RTDTemperature[0] / 100.0f;
                float  left_temp = (float)RTDTemperature[1] / 100.0f;

                rk3576_uart_port.sender(DATA_FLOAT, F32_LEFT_TEMP_VALUE,  &left_temp);
                rk3576_uart_port.sender(DATA_FLOAT, F32_RIGHT_TEMP_VALUE, &right_temp);
            }
        }

        // ===========================================================
        // 【4】异常/安全状态管理
        // ===========================================================
        if (WorkMode == 0x00)
        {
            // 系统空闲，可进入待机或低功耗模式
        }
        else if (WorkMode == 0xFF)
        {
            // 锁定状态：可在此执行保护停机
        }
        else if (WorkMode > 0x0F)
        {
            WorkMode = 0xFF; // 防止非法状态
        }

        // ===========================================================
        // 【5】错误检测 / 告警上报（500ms一次，可选）
        // ===========================================================
        if (timer_500ms == 0)
        {
            uint8_t alarm_state = 0;
            if (RTDTemperature[0] > 6000 || RTDTemperature[1] > 6000)
                alarm_state |= (1 << 0); // 温度过高
            if (left_pressure > 40000 || right_pressure > 40000)
                alarm_state |= (1 << 1); // 压力过高
            if (WaterSensor == 0)
                alarm_state |= (1 << 2); // 无水告警

            rk3576_uart_port.sender(DATA_UINT8_T, U8_ALARM_STATE, &alarm_state);
        }

        // ===========================================================
        // 周期延时与计数更新
        // ===========================================================
        vTaskDelay(pdMS_TO_TICKS(10));
        timer_100ms++;
        timer_500ms++;
        timer_5000ms++;
    }
}


