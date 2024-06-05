#include "car.h"
#include <stdint.h>
#include "tim.h"

t_CAR car;	//定义一辆小车

void Set_Motor(int16_t Motor_A,int16_t Motor_B)	//设置电机的转向和占空比
{
	if(Motor_A>0)
	{
		AIN1_SET;
		AIN2_RESET;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, Motor_A);
	}
	else if(Motor_A<0)
	{
		AIN1_RESET;
		AIN2_SET;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, -Motor_A);
	}
	else if(Motor_A==0)
	{
		AIN1_RESET;
		AIN2_RESET;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_4, Motor_A);
	}
	if(Motor_B>0)
	{
		BIN1_SET;
		BIN2_RESET;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, Motor_B);
	}
	else if(Motor_B<0)
	{
		BIN1_RESET;
		BIN2_SET;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3, -Motor_B);
	}
	else if(Motor_B==0)
	{
		BIN1_RESET;
		BIN2_RESET;
		__HAL_TIM_SetCompare(&htim4, TIM_CHANNEL_3,Motor_B);
	}
}


//获取小车转速和速度
void Get_All_Speed(t_CAR* m_car,int16_t time_IT)
{
	m_car->A.rospeed=(Encoder_Get_A()*60*time_IT/1040);	//单位：r/min
	m_car->B.rospeed=(Encoder_Get_B()*60*time_IT/1040);	//单位：r/min
	m_car->A.velocity=m_car->A.rospeed*0.045*3.14/60;	//单位：m/s
	m_car->B.velocity=m_car->B.rospeed*0.045*3.14/60;	//单位：m/s
}

void pid_Init(t_CAR* m_car)
{
	/*******PID参数********/
	m_car->A.PID.Kp=0.80;
	m_car->A.PID.Ki=0.05;
	m_car->A.PID.Kd=0.05;
	m_car->B.PID.Kp=0.80;
	m_car->B.PID.Ki=0.05;
	m_car->B.PID.Kd=0.05;
	/*******************/
	m_car->A.PID.target_val=50;
	m_car->B.PID.target_val=50;
	/*****************/
	m_car->A.PID.err=0;
	m_car->A.PID.err_last=0;
	m_car->A.PID.err_sum=0;
	m_car->B.PID.err=0;
	m_car->B.PID.err_last=0;
	m_car->B.PID.err_sum=0;
}

void PID(t_CAR* m_car)
{
	/*********A**********/
	m_car->A.PID.actual_val=m_car->A.rospeed;
	m_car->A.PID.err=m_car->A.PID.target_val-m_car->A.PID.actual_val;
	m_car->A.PID.err_sum+=m_car->A.PID.err;
	m_car->A.PID.output_val=m_car->A.PID.Kp*m_car->A.PID.err+m_car->A.PID.Ki*m_car->A.PID.err_sum+m_car->A.PID.Kd*(m_car->A.PID.err-m_car->A.PID.err_last);
	m_car->A.PID.err_last=m_car->A.PID.err;
	m_car->A.PWM=m_car->A.PID.output_val;
	/*********B**********/
	m_car->B.PID.actual_val=m_car->B.rospeed;
	m_car->B.PID.err=m_car->B.PID.target_val-m_car->B.PID.actual_val;
	m_car->B.PID.err_sum+=m_car->B.PID.err;
	m_car->B.PID.output_val=m_car->B.PID.Kp*m_car->B.PID.err+m_car->B.PID.Ki*m_car->B.PID.err_sum+m_car->B.PID.Kd*(m_car->B.PID.err-m_car->B.PID.err_last);
	m_car->B.PID.err_last=m_car->B.PID.err;
	m_car->B.PWM=m_car->B.PID.output_val;
}
