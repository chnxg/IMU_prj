#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f10x.h"
#include "mpu6050.h"

#define RtA 	57.324841				//弧度到角度
#define AtR    	0.0174533				//度到角度
#define Acc_G 	0.0011963				//加速度变成G
#define Gyro_G 	0.03051756		  //角速度变成度
#define Gyro_Gr	0.0005326	

#define FILTER_NUM 10

typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_ANGLE;			//四元数计算出的角度

extern S_INT16_XYZ ACC_AVG;			//平均值滤波后的ACC
void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float, float, float, float, float, float, float, float, float);

#endif
