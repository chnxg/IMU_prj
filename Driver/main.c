#include"stm32f10x.h"
#include "led.h"
#include "usart.h"
#include "mpu6050.h"
#include "hmc5883.h"
#include "myi2c.h"
#include "IMU.h"
#include "oled.h"

#define CLI()      __set_PRIMASK(1)  //关总中断                                               
#define SEI()      __set_PRIMASK(0)	 //开总中断

void Delay(__IO u32 nCount);

int16_t count;


int main() 
{   
  CLI(); //关总中断
  Delay(0xFFFEF);
  LED_GPIO_Config();
  I2C_GPIO_Config();
  Delay(0xFFFF);
  InitMPU6050();
  Delay(0xFFFF);
  Init_HMC5883L();
  Tim4_Init(2000);	//2000=2ms
  Nvic_Init();
  usart1_config();
  TIM2_PWM_Init();
  SEI(); //开总中断
  while(1)
  {

  }   
}

void Delay(__IO u32 nCount)
{
  for(; nCount != 0; nCount--);
} 




