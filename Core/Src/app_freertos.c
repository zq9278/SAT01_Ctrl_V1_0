/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "heat.h"
#include "ads1248.h"
#include "Pressure_sensor.h"
#include "apply.h"
#include "ds18b20.h"
#include "water.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
volatile u8 OTP1_RESET_Value,OTP2_RESET_Value;
volatile u16 RTDTemperature[2];
volatile s16 WaterTemperature;
volatile u8 WaterSensor;

extern volatile u8 WaterState;

extern volatile u8 WorkMode;

extern PID_TypeDef RightHeat,LeftHeat;
/*
0x00 ����
0x01 ���ۼ�ѹ

0xFF ����ǰ�ر�����
*/

extern volatile RingBuffer_t uart_ring_buffer;
extern volatile uint8_t uart_rx_byte;
extern u16 left_pressure, right_pressure;


QueueHandle_t frameQueue;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId LED_BLINKHandle;
osThreadId OTP_READHandle;
osThreadId RTDReadHandle;
osThreadId FrameParseHandle;
osThreadId FrameHandlerHandle;
osThreadId PressureReadHandle;
osThreadId WorkModeTaskHandle;
osThreadId LowFreq_TaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
int RingBuffer_GetByte(uint8_t *out_byte) {
    if (uart_ring_buffer.head == uart_ring_buffer.tail) {
        return 0;  // ������Ϊ��
    }

    *out_byte = uart_ring_buffer.buffer[uart_ring_buffer.tail];
    uart_ring_buffer.tail = (uart_ring_buffer.tail + 1) % UART_RX_BUF_SIZE;
    return 1;
}


/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void vTaskLEDBlink(void const * argument);
void vTaskOTPRead(void const * argument);
void vTaskRTDRead(void const * argument);
void vTaskFrameParse(void const * argument);
void vTaskFrameHandler(void const * argument);
void vTaskPressureRead(void const * argument);
void vTaskWorkMode(void const * argument);
void vTaskLowFreq(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LED_BLINK */
  osThreadDef(LED_BLINK, vTaskLEDBlink, osPriorityIdle, 0, 128);
  LED_BLINKHandle = osThreadCreate(osThread(LED_BLINK), NULL);

  /* definition and creation of OTP_READ */
  osThreadDef(OTP_READ, vTaskOTPRead, osPriorityIdle, 0, 128);
  OTP_READHandle = osThreadCreate(osThread(OTP_READ), NULL);

  /* definition and creation of RTDRead */
  osThreadDef(RTDRead, vTaskRTDRead, osPriorityIdle, 0, 128);
  RTDReadHandle = osThreadCreate(osThread(RTDRead), NULL);

  /* definition and creation of FrameParse */
  osThreadDef(FrameParse, vTaskFrameParse, osPriorityLow, 0, 128);
  FrameParseHandle = osThreadCreate(osThread(FrameParse), NULL);

  /* definition and creation of FrameHandler */
  osThreadDef(FrameHandler, vTaskFrameHandler, osPriorityIdle, 0, 128);
  FrameHandlerHandle = osThreadCreate(osThread(FrameHandler), NULL);

  /* definition and creation of PressureRead */
  osThreadDef(PressureRead, vTaskPressureRead, osPriorityIdle, 0, 128);
  PressureReadHandle = osThreadCreate(osThread(PressureRead), NULL);

  /* definition and creation of WorkModeTask */
  osThreadDef(WorkModeTask, vTaskWorkMode, osPriorityIdle, 0, 256);
  WorkModeTaskHandle = osThreadCreate(osThread(WorkModeTask), NULL);

  /* definition and creation of LowFreq_Task */
  osThreadDef(LowFreq_Task, vTaskLowFreq, osPriorityIdle, 0, 128);
  LowFreq_TaskHandle = osThreadCreate(osThread(LowFreq_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
	frameQueue = xQueueCreate(10, sizeof(ProtocolFrame_t));
	if (frameQueue == NULL) {
//			Error_Handler();  // �����Զ���һ���ϵ��LED����
	}
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
		osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_vTaskLEDBlink */
/**
* @brief Function implementing the LED_BLINK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLEDBlink */
void vTaskLEDBlink(void const * argument)
{
  /* USER CODE BEGIN vTaskLEDBlink */
  /* Infinite loop */
  for(;;)
  {
		
		HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		
    osDelay(500);
  }
  /* USER CODE END vTaskLEDBlink */
}

/* USER CODE BEGIN Header_vTaskOTPRead */
/**
* @brief Function implementing the OTP_READ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskOTPRead */
void vTaskOTPRead(void const * argument)
{
  /* USER CODE BEGIN vTaskOTPRead */
  /* Infinite loop */
  for(;;)
  {
		
		OTP1_RESET_Value=HAL_GPIO_ReadPin(OTP1_RESET_GPIO_Port,OTP1_RESET_Pin);
		OTP2_RESET_Value=HAL_GPIO_ReadPin(OTP2_RESET_GPIO_Port,OTP2_RESET_Pin);
    osDelay(500);
  }
  /* USER CODE END vTaskOTPRead */
}

/* USER CODE BEGIN Header_vTaskRTDRead */
/**
* @brief Function implementing the RTDRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskRTDRead */
void vTaskRTDRead(void const * argument)
{
  /* USER CODE BEGIN vTaskRTDRead */
	u8 RTDChannel=RTD1;
  /* Infinite loop */
  for(;;)
  {
		if(HAL_GPIO_ReadPin(RTD_RDY_GPIO_Port,RTD_RDY_Pin)==GPIO_PIN_RESET)
		{
			RTDTemperature[RTDChannel]=ADC2Temperature(ADS1248_Read());
//		temp3=ADC2Temperature(RTDTemperature[RTDChannel]);
			RTDChannel=1^RTDChannel;
			ADS1248_ChangeChannel(RTDChannel);
		}
    osDelay(30);
  }
  /* USER CODE END vTaskRTDRead */
}

/* USER CODE BEGIN Header_vTaskFrameParse */
/**
* @brief Function implementing the FrameParse thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskFrameParse */
void vTaskFrameParse(void const * argument)
{
  /* USER CODE BEGIN vTaskFrameParse */
	uint8_t parse_buffer[UART_RX_BUF_SIZE];
  uint16_t parse_index = 0;
  /* Infinite loop */
  for(;;)
  {
     uint8_t byte;
        if (RingBuffer_GetByte(&byte)) {
            parse_buffer[parse_index++] = byte;

            while (parse_index >= 5) // ����֡ͷ(2)+����(1)+���(1)+ID(2)+CRC(2)
            {
                if (parse_buffer[0] != 0x5A || parse_buffer[1] != 0xA5) {
                    memmove(parse_buffer, &parse_buffer[1], --parse_index);
                    continue;
                }

                uint8_t len = parse_buffer[2];
                uint16_t full_frame_len = 3 + len;

                if (parse_index < full_frame_len) break; // ���ݻ�û����

                // CRC У��
                uint16_t calc_crc = ModbusCRC16(&parse_buffer[3], len - 2);  // ���� CRC �����ǰ len-2 �ֽ�
                uint16_t recv_crc = (parse_buffer[3 + len - 1] << 8) | parse_buffer[3 + len - 2];  // ע�⣺Modbus CRC ���ֽ���ǰ��

                if (calc_crc == recv_crc) {
                    // �Ϸ�֡
                    ProtocolFrame_t frame;
                    frame.frame_id[0] = parse_buffer[4];
                    frame.frame_id[1] = parse_buffer[5];
                    frame.payload_len = len - 2 - 3;  // ��ȥ CRC2�ֽ� �� ID+���3�ֽ�
                    frame.payload = pvPortMalloc(frame.payload_len);
                    memcpy(frame.payload, &parse_buffer[6], frame.payload_len);
                    xQueueSend(frameQueue, &frame, 0);
                }

                // �Ƴ��Ѵ���֡
                parse_index -= full_frame_len;
                memmove(parse_buffer, &parse_buffer[full_frame_len], parse_index);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(2));
        }
  }
  /* USER CODE END vTaskFrameParse */
}

/* USER CODE BEGIN Header_vTaskFrameHandler */
/**
* @brief Function implementing the FrameHandler thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskFrameHandler */
void vTaskFrameHandler(void const * argument)
{
  /* USER CODE BEGIN vTaskFrameHandler */
	 ProtocolFrame_t frame;
  /* Infinite loop */
  for(;;)
  {
		if (xQueueReceive(frameQueue, &frame, portMAX_DELAY) == pdPASS) {
            FrameDispatcher(&frame);  // �� �ַ�����
        }
  }
  /* USER CODE END vTaskFrameHandler */
}

/* USER CODE BEGIN Header_vTaskPressureRead */
/**
* @brief Function implementing the PressureRead thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskPressureRead */
void vTaskPressureRead(void const * argument)
{
  /* USER CODE BEGIN vTaskPressureRead */
  /* Infinite loop */
  for(;;)
  {
		pressure_sensor_read();
    osDelay(100);
  }
  /* USER CODE END vTaskPressureRead */
}

/* USER CODE BEGIN Header_vTaskWorkMode */
/**
* @brief Function implementing the WorkModeTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskWorkMode */
void vTaskWorkMode(void const * argument)
{
  /* USER CODE BEGIN vTaskWorkMode */
	u8 timer_100ms,timer_500ms;
	u16 timer_5000ms;
	u8 ReturnData;
	
  /* Infinite loop */
  for(;;)
  {
		if(timer_100ms==10)timer_100ms=0;
		if(timer_500ms==50)timer_500ms=0;
		if(timer_5000ms==500)timer_5000ms=0;
		if(timer_5000ms==0)
		{
			SendWaterState(WaterState,WaterSensor,WaterTemperature);
		}
		
		if(WorkMode==0x00)
		{
			
		}
		else if(WorkMode<=0x0F)
		{
			if((WorkMode&0x03)!=0)//��ѹ
			{
				Work_Expression(WorkMode&0x03);
				if(timer_100ms==0)
				{
					SendPressure(right_pressure,left_pressure);
				}
			}
			if((WorkMode&0x0C)!=0)//����
			{
				Work_Heat((WorkMode&0x0C)>>2);
				if(timer_100ms==0)
				{
					SendEyeTemperature(RTDTemperature[0],RTDTemperature[1]);
				}
			}
		}
		else if(WorkMode<=0x3F)
		{
			if((WorkMode&0x10)==0x10)//��ˮ
			{
				ReturnData=Work_DrainWater();
				if(ReturnData!=2)
				{
					SendDrainWaterState(ReturnData);
				}
			}
			else if((WorkMode&0x20)==0x20)//��ˮ
			{
				ReturnData=Work_AddWater();
				if(ReturnData!=2)
				{
					SendAddWaterState(ReturnData);
				}
			}
		}
		else if(WorkMode==0xFF)
		{
			
		}
		else
		{
			WorkMode=0xFF;
		}
		
    osDelay(10);
		timer_100ms++;
		timer_500ms++;
		timer_5000ms++;
  }
  /* USER CODE END vTaskWorkMode */
}

/* USER CODE BEGIN Header_vTaskLowFreq */
/**
* @brief Function implementing the LowFreq_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_vTaskLowFreq */
void vTaskLowFreq(void const * argument)
{
  /* USER CODE BEGIN vTaskLowFreq */
  /* Infinite loop */
  for(;;)
  {
		WaterTemperature=DS18B20_Get_Temp();
		WaterSensor=HAL_GPIO_ReadPin(WS1_GPIO_Port,WS1_Pin)^ 0x01;
    osDelay(5000);
  }
  /* USER CODE END vTaskLowFreq */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

