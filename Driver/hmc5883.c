#include "hmc5883.h"
#include "myi2c.h"
#include  <math.h>

uint8_t BUF[8];
double angle;
int16_t X_HMC,Y_HMC,Z_HMC,x,y,z;
double y_gain=0.967;

int16_t x_offest,y_offest,z_offest;
u16 cal_cnt=5000;
vs16 mx_min=0,mx_max=0,my_min=0,my_max=0,mz_min=0,mz_max=0;


//******************************************************
//连续读出HMC5883内部角度数据，地址范围0x3~0x5
//******************************************************
void Multiple_Read_HMC5883L(void)
{      	
	u8 i,a;

    I2C_Start();                          //起始信号
    I2C_SendByte(HMC5883L_ADDRESS);                   //发送设备地址+写信号
	I2C_WaitAck();
    I2C_SendByte(0x03);                   //发送存储单元地址，从0x3开始	
	I2C_WaitAck();
    I2C_Start();                          //起始信号
    I2C_SendByte(HMC5883L_ADDRESS+1);     //发送设备地址+读信号
	I2C_WaitAck();
	for (i=0; i<6; i++)                   //连续读取6个地址数据，存储中BUF
    {
        BUF[i] = I2C_RadeByte();          //BUF[0]存储数据
        if (i == 5)
           I2C_NoAck();                   //最后一个数据需要回NOACK
        else
           I2C_Ack();                     //回应ACK
    }
    I2C_Stop();                           //停止信号
    delay5ms();

	x=BUF[0] << 8 | BUF[1]; //Combine MSB and LSB of X Data output register
	z=BUF[2] << 8 | BUF[3]; //Combine MSB and LSB of Z Data output register
	y=BUF[4] << 8 | BUF[5]; //Combine MSB and LSB of Y Data output register

		
	if(cal_cnt)	
	{
    cal_cnt--;
		if(mx_min>x)	mx_min = x;
		if(mx_max<x)	mx_max = x;
		if(my_min>y)	my_min = y;
		if(my_max<y)	my_max = y;
		if(mz_min>z)	mz_min = z;
		if(mz_max<z)	mz_max = z;
		
		if(cal_cnt==0)
		{
			x_offest = (mx_min+mx_max)/2;
			y_offest = (my_min+my_max)/2;
			z_offest = (mz_min+mz_max)/2;
		}
  }

  X_HMC = 1 *(x +1);
  Y_HMC = (double)(1 * (y - 1));
	Z_HMC = (double)(1 * (z +1));	
		
}	   
 
void Init_HMC5883L()
{
  Single_Write(HMC5883L_ADDRESS,0x02,0x00); 


}

