/*
 * @Author: zhangqi zq9278@gmail.com
 * @Date: 2024-06-15 13:31:10
 * @LastEditors: zhangqi zq9278@gmail.com
 * @LastEditTime: 2024-07-01 18:14:23
 * @FilePath: \EIDEd:\Project\SLK01\Software\SLK-01-new\MDK-ARM\USER\pid\pid.c
 * @Description: 
 * 
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved. 
 */
// pid.c

#include "main.h"
#include "pid.h"

int32_t p, i, d;

// ΢���˲�ϵ����0 < alpha < 1����Խ��Խƽ��
#define DERIVATIVE_FILTER_ALPHA 0.8f

//void PID_Init(PID_TypeDef *pid, float Kp, float Ki, float Kd,
//              float integral_max, float integral_min,
//              float output_max, float output_min, float setpoint) {
//    pid->Kp = Kp;
//    pid->Ki = Ki;
//    pid->Kd = Kd;
//    pid->previous_measured_value = 0.0f;   // ����΢������
//    pid->integral = 0.0f;
//    pid->derivative_filtered = 0.0f;       // ΢���˲���ֵ
//    pid->integral_max = integral_max;
//    pid->integral_min = integral_min;
//    pid->output_max = output_max;
//    pid->output_min = output_min;
//    pid->setpoint = setpoint;
//}
							


//float PID_Compute(PID_TypeDef *pid, float measured_value) {
//    // �������
//    float error = pid->setpoint - measured_value;

//    // ��������㲢�޷�
//    pid->integral += error;
//    pid->integral = Limit(pid->integral, pid->integral_min, pid->integral_max);

//    // ? ΢������ڲ���ֵ������
//    float derivative = measured_value - pid->previous_measured_value;

//    // ? ΢���˲���������ͨ�˲���΢����
//    pid->derivative_filtered = DERIVATIVE_FILTER_ALPHA * pid->derivative_filtered +
//                               (1.0f - DERIVATIVE_FILTER_ALPHA) * derivative;

//    // ���浱ǰ����ֵ�����´�΢�ּ���
//    pid->previous_measured_value = measured_value;

//    // PID �������
//    p = pid->Kp * error;
//    i = pid->Ki * pid->integral;
//    d = -pid->Kd * pid->derivative_filtered;  // ע����ţ�΢�����������Ʋ���ֵͻ��

//    // ����������޷�
//    float output = p + i + d;
//    float output_limited = Limit(output, pid->output_min, pid->output_max);

//    // �������
//    printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",p,i,d,measured_value,output_limited,pid->setpoint);

//    return output_limited;
//}


void PID_Init(PID_TypeDef *pid, int32_t Kp, int32_t Ki, int32_t Kd,
                  int32_t integral_max, int32_t integral_min,
                  int32_t output_max, int32_t output_min, int32_t setpoint) 
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->setpoint = setpoint;
    pid->integral = 0;
    pid->derivative_filtered = 0;
    pid->previous_measured_value = 0;

    pid->integral_max = integral_max;
    pid->integral_min = integral_min;

    pid->output_max = output_max;
    pid->output_min = output_min;
}
									
int32_t PID_Compute(PID_TypeDef *pid, int32_t measured_value) 
{
    int32_t error = pid->setpoint - measured_value;

    // ���ּ��㼰�޷�
    pid->integral += error;
    if (pid->integral > pid->integral_max) pid->integral = pid->integral_max;
    else if (pid->integral < pid->integral_min) pid->integral = pid->integral_min;

    // ΢�֣����ڲ���ֵ�仯��+ ��ͨ�˲�
    int32_t derivative = measured_value - pid->previous_measured_value;
//    pid->derivative_filtered = (DERIVATIVE_FILTER_ALPHA_NUM * pid->derivative_filtered +
//                                (DERIVATIVE_FILTER_ALPHA_DEN - DERIVATIVE_FILTER_ALPHA_NUM) * derivative)
//                               / DERIVATIVE_FILTER_ALPHA_DEN;
    pid->previous_measured_value = measured_value;

    // PID ������㣨��ϵ���Ŵ�1000������Ҫ����ȥ��
    p = (pid->Kp * error) / 1000;
    i = (pid->Ki * pid->integral) / 1000;
    d = -(pid->Kd) / 1000;//-(pid->Kd * pid->derivative_filtered) / 1000;

    int32_t output = p + i + d;

    // ����޷�
    if (output > pid->output_max) output = pid->output_max;
    else if (output < pid->output_min) output = pid->output_min;

    // �����������ѡ��
//    printf("P:%d I:%d D:%d M:%d OUT:%d SP:%d\n", p, i, d, measured_value, output, pid->setpoint);

    return output;
}
