//
// Created by zq on 2025/5/21.
//

#ifndef ELECTRICAL_MUSCLE_QUBEMX_LOG_H
#define ELECTRICAL_MUSCLE_QUBEMX_LOG_H
#include "AppMain.h"
#include <stdarg.h>
#include <stdio.h>       // ? printf 系列
#include "stdint.h"
#include "FreeRTOS.h"   // FreeRTOS 基础头文件
#include "semphr.h"     // 用于使用 SemaphoreHandle_t 类型
#include "queue.h"      // 用于使用 QueueHandle_t 类型
// 日志等级枚举（可扩展用作过滤）
typedef enum {
    LOG_LEVEL_INFO,
    LOG_LEVEL_WARN,
    LOG_LEVEL_ERROR
} log_level_t;

#define LOG_BUF_LEN 200
#define LOG_QUEUE_LEN 5

typedef struct {
    char buf[LOG_BUF_LEN];
    size_t len;
} LogMessage_t;

extern QueueHandle_t logQueue;

// 初始化函数，传入 UART 句柄
void LOG_Init(void);

// 主日志输出接口（线程安全封装 vprintf）
void LOG(const char *format, ...);
extern SemaphoreHandle_t logSemaphore;

// 快捷宏定义（带换行）
#define LOG_I(fmt, ...) LOG("[I] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_W(fmt, ...) LOG("[W] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_E(fmt, ...) LOG("[E] " fmt "\r\n", ##__VA_ARGS__)
void LOG_ISR(const char *format, ...);
#endif //ELECTRICAL_MUSCLE_QUBEMX_LOG_H