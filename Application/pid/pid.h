// pid.h

#ifndef PID_H
#define PID_H
#include "stm32g0xx_hal.h"
typedef struct {
    int32_t Kp;
    int32_t Ki;
    int32_t Kd;
    int32_t previous_error;
    int32_t integral;
    int32_t integral_max;   // �����޷�ֵ
    int32_t integral_min;   // �������޷�ֵ
    int32_t output_max;     // ����޷�ֵ
    int32_t output_min;     // ������޷�ֵ
    int32_t setpoint;       // �趨ֵ
    int32_t previous_measured_value ;
    int32_t derivative_filtered ;
} PID_TypeDef;

// ��ʼ��PID������
void PID_Init(PID_TypeDef *pid, int32_t Kp, int32_t Ki, int32_t Kd,
                  int32_t integral_max, int32_t integral_min,
                  int32_t output_max, int32_t output_min, int32_t setpoint) ;

// ����PID�������
int32_t PID_Compute(PID_TypeDef *pid, int32_t measured_value);

// �޷���
//#define Limit(x, min, max) ((x) = (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x))))
#define Limit(x, min, max) (((x) <= (min)) ? (min) : (((x) >= (max)) ? (max) : (x)))

#endif // PID_H
