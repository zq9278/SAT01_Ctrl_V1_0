#include "water.h"

extern TIM_HandleTypeDef htim15;

volatile u8 WaterState=0;
/*
0000 0xyz
x:0,δ�ڼ�ˮ
  1,���ڼ�ˮ
y:0��δ����ˮ
  1,������ˮ
z:0��δ��ѭ��
  1������ѭ��
*/

u8 StartWaterPump(void)
{
	if(HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)==0)
	{
		WaterPump1(1);
		WaterState=WaterState|0x01;
		return 1;
	}
	else
	{
		return 0;
	}
}

void StopWaterPump(void)
{
	WaterPump1(0);
	WaterState=WaterState&0xfe;
}

void StartIPLCold(u8 ColdValue)
{
	HAL_TIM_PWM_Start(&htim15,TIM_CHANNEL_2);
	if(ColdValue>190)ColdValue=190;
	TIM15->CCR2=ColdValue;
}

void StopIPLCold(void)
{
	HAL_TIM_PWM_Stop(&htim15,TIM_CHANNEL_2);
}
