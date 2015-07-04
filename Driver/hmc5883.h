#ifndef _HMC5883_H
#define _HMC5883_H
#include "stm32f10x.h"
#define	HMC5883L_ADDRESS   0x3C	  //定义器件在IIC总线中的从地址

extern void Init_HMC5883L(void);
extern void Multiple_Read_HMC5883L(void);
extern uint8_t BUF[8];
extern double angle;
extern int16_t X_HMC,Y_HMC,Z_HMC,x,y,z;
#endif

