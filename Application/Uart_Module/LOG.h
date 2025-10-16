//
// Created by zq on 2025/5/21.
//

#ifndef ELECTRICAL_MUSCLE_QUBEMX_LOG_H
#define ELECTRICAL_MUSCLE_QUBEMX_LOG_H
#include "AppMain.h"
#include <stdarg.h>
#include <stdio.h>       // ? printf ϵ��
#include "stdint.h"
#include "FreeRTOS.h"   // FreeRTOS ����ͷ�ļ�
#include "semphr.h"     // ����ʹ�� SemaphoreHandle_t ����
#include "queue.h"      // ����ʹ�� QueueHandle_t ����
// ��־�ȼ�ö�٣�����չ�������ˣ�
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

// ��ʼ������������ UART ���
void LOG_Init(void);

// ����־����ӿڣ��̰߳�ȫ��װ vprintf��
void LOG(const char *format, ...);
extern SemaphoreHandle_t logSemaphore;

// ��ݺ궨�壨�����У�
#define LOG_I(fmt, ...) LOG("[I] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_W(fmt, ...) LOG("[W] " fmt "\r\n", ##__VA_ARGS__)
#define LOG_E(fmt, ...) LOG("[E] " fmt "\r\n", ##__VA_ARGS__)
void LOG_ISR(const char *format, ...);
#endif //ELECTRICAL_MUSCLE_QUBEMX_LOG_H