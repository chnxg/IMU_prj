#ifndef _IMU_H_
#define _IMU_H_
#include "stm32f10x.h"
#include "mpu6050.h"

#define RtA 	57.324841				//���ȵ��Ƕ�
#define AtR    	0.0174533				//�ȵ��Ƕ�
#define Acc_G 	0.0011963				//���ٶȱ��G
#define Gyro_G 	0.03051756		  //���ٶȱ�ɶ�
#define Gyro_Gr	0.0005326	

#define FILTER_NUM 10

typedef struct{
				float X;
				float Y;
				float Z;}S_FLOAT_XYZ;
extern S_FLOAT_XYZ Q_ANGLE;			//��Ԫ��������ĽǶ�

extern S_INT16_XYZ ACC_AVG;			//ƽ��ֵ�˲����ACC
void Prepare_Data(void);
void Get_Attitude(void);
void IMUupdate(float, float, float, float, float, float, float, float, float);

#endif
