#include "motor.h"
#include "imu.h"
#include "mpu6050.h"


//static float  Kpn = 0.01;      //PID参数
//static float  Ksp = 2.0;	    //PID参数

void PWM_Motor(int16_t L, int16_t R)	 
{  
	  if(L>990)
	    L=990;
	  else if(L<-990)
	    L=-990;
	  if(R>990)
	    R=990;
	  else if(R<-990)
	    R=-990;

	 if(L>=0)	//13   24        14qian   23hou
	 {
	  TIM2->CCR3 = 0;
	  TIM2->CCR1 = L;
	 }
	 else
	 {
	  TIM2->CCR3 = -L; 
	  TIM2->CCR1 = 0;
	 }
	
	 if(R>=0)
	 {
	  TIM2->CCR2 = 0;  
	  TIM2->CCR4 = R;
	 }
	 else
	 {
	  TIM2->CCR2 = -R;  
	  TIM2->CCR4 = 0;
	 }	 
}

int16_t PWM_L,PWM_R,MOTO_temp_pwm_1,MOTO_temp_pwm_2,MOTO_temp_pwm_old;
int16_t goal_speed,MOTO_go_pwm,MOTO_bk_pwm;
int16_t error_speed_1[3]={0,0,0};      //实际测得的脉冲个数      
int16_t error_speed_2[3]={0,0,0};       //期望的脉冲个数与实际脉冲测得脉冲个数的偏差
float  Speed_P=0; 
float  Speed_I=1; 
float  Speed_D=0;  //PID参数
float  Kp  = 100;       //PID参数
float  Kd  = 0;	    //PID参数
extern s16 speed_1,speed_2;
int16_t now_speed;

void PWM_Calcu(void)	 
{  
	goal_speed = Kp*Q_ANGLE.Y + Kd*MPU6050_GYRO_LAST.Y;          //PID：角速度和角度
	//goal_speed=100;
//	now_speed = (speed_1 + speed_2)/2;

	error_speed_1[2]=error_speed_1[1]; error_speed_1[1]=error_speed_1[0]; error_speed_1[0]=goal_speed - speed_1;                 
    MOTO_temp_pwm_1 += Speed_P*(error_speed_1[0]-error_speed_1[1]) + Speed_I*error_speed_1[0] + Speed_D*(error_speed_1[0]-2*error_speed_1[1]+error_speed_1[2]);
 
    if(MOTO_temp_pwm_1 > 990)  MOTO_temp_pwm_1 = 990;   //占空比上限限幅       
    else if(MOTO_temp_pwm_1 < -990)   MOTO_temp_pwm_1 = -990;  //占空比下限限幅  


	error_speed_2[2]=error_speed_2[1]; error_speed_2[1]=error_speed_2[0]; error_speed_2[0]=goal_speed - speed_2;                 
    MOTO_temp_pwm_2 += Speed_P*(error_speed_2[0]-error_speed_2[1]) + Speed_I*error_speed_2[0] + Speed_D*(error_speed_2[0]-2*error_speed_2[1]+error_speed_2[2]);
 
    if(MOTO_temp_pwm_2 > 990)  MOTO_temp_pwm_2 = 990;   //占空比上限限幅       
    else if(MOTO_temp_pwm_2 < -990)   MOTO_temp_pwm_2 = -990;  //占空比下限限幅  

//	PWM += Kpn*position + Ksp*speed;      //PID：速度和位置
	PWM_L = MOTO_temp_pwm_1;// + turn_need;
	PWM_R = MOTO_temp_pwm_2;// - turn_need;
	PWM_Motor(PWM_L,PWM_R); 	 
}