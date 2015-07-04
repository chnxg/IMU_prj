/******************** (C) COPYRIGHT  **************************
 * �ļ���  ��main.c       
 * ʵ��ƽ̨��Beyond�Ƽ�IMU 
 * ��汾  ��ST3.5.0
 * �Ա�    ��http://beyond.taobao.com
**********************************************************************************/
#include "stm32f10x.h"
#include "led.h"
#include "usart.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "myi2c.h"
#include "IMU.h"
#include "tim.h"

extern u8 sentFlag;	
void Delay(__IO u32 nCount);
/* 
 * ��������main
 * ����  : ������
 * ����  ����
 * ���  : ��
 */
int main(void)
{
	/* ����ϵͳʱ��Ϊ 72M */ 
	SystemInit(); 
	LED_GPIO_Config();
	Nvic_Init();
	I2C_GPIO_Config();
	Delay(0xFFFF);
	InitMPU6050();
	Delay(0xFFFF);
  Init_HMC5883L();
	Delay(0xFFFF);
	Tim3_Init(2500);
	usart1_config();
	
  while(1)
  {
	    if(sentFlag)
			{
				 sentFlag = 0;								 
			   UART1_ReportIMU();
			}
	}
	  
}
/******************* (C) COPYRIGHT  *****END OF FILE************/

void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 
