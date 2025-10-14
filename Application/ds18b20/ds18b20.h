#ifndef __DS18B20_H
#define __DS18B20_H
#include "sys.h"

//IO��������
#define DS18B20_IO_IN()  {GPIOB->MODER&=~(GPIO_MODER_MODE0<<(5*2));GPIOB->MODER|=0<<5*2;}	//PB5����ģʽ
#define DS18B20_IO_OUT() {GPIOB->MODER&=~(GPIO_MODER_MODE0<<(5*2));GPIOB->MODER|=1<<5*2;} 	//PB5���ģʽ
 
//IO��������											   
#define	DS18B20_DQ_OUT(n) (n?HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_SET):HAL_GPIO_WritePin(GPIOB,GPIO_PIN_5,GPIO_PIN_RESET)) //���ݶ˿�	PB5
#define	DS18B20_DQ_IN  HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_5) //���ݶ˿�	PB5
   	
u8 DS18B20_Init(void);			//��ʼ��DS18B20
short DS18B20_Get_Temp(void);	//��ȡ�¶�
void DS18B20_Start(void);		//��ʼ�¶�ת��
void DS18B20_Write_Byte(u8 dat);//д��һ���ֽ�
u8 DS18B20_Read_Byte(void);		//����һ���ֽ�
u8 DS18B20_Read_Bit(void);		//����һ��λ
u8 DS18B20_Check(void);			//����Ƿ����DS18B20
void DS18B20_Rst(void);			//��λDS18B20 
#endif
