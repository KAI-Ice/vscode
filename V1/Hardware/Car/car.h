#ifndef __CAR_H__
#define __CAR_H__
#include <stdint.h>
#include "main.h"

/****************定义小车的结构参数*********************/
typedef struct
{
	float Kp,Ki,Kd;
	float target_val,actual_val,change_val,output_val;
	float err,err_last,err_lastlast,err_sum;
}t_PID;	//一个电机的PID

typedef struct
{
	t_PID PID;
	int16_t PWM;	//有方向的PWM，为正数前滚，为负数后滚
	int16_t rospeed;
	float velocity;
}t_MOTOR;	//一个电机

typedef struct
{
	t_MOTOR A,B;	//小车只有两个电机
	
}t_CAR;	//一辆小车
/******************************************************/
//定义电机正反转控制引脚，AIN1和AIN2各为正负即为正转，BIN1和BIN2各为正负即为正转
#define AIN1_SET		HAL_GPIO_WritePin(Car_AIN1_GPIO_Port,Car_AIN1_Pin, GPIO_PIN_SET);
#define AIN1_RESET		HAL_GPIO_WritePin(Car_AIN1_GPIO_Port,Car_AIN1_Pin, GPIO_PIN_RESET);
#define AIN2_SET		HAL_GPIO_WritePin(Car_AIN2_GPIO_Port,Car_AIN2_Pin, GPIO_PIN_SET);
#define AIN2_RESET		HAL_GPIO_WritePin(Car_AIN2_GPIO_Port,Car_AIN2_Pin, GPIO_PIN_RESET);
#define BIN1_SET		HAL_GPIO_WritePin(Car_BIN1_GPIO_Port,Car_BIN1_Pin, GPIO_PIN_SET);
#define BIN1_RESET		HAL_GPIO_WritePin(Car_BIN1_GPIO_Port,Car_BIN1_Pin, GPIO_PIN_RESET);
#define BIN2_SET		HAL_GPIO_WritePin(Car_BIN2_GPIO_Port,Car_BIN2_Pin, GPIO_PIN_SET);
#define BIN2_RESET		HAL_GPIO_WritePin(Car_BIN2_GPIO_Port,Car_BIN2_Pin, GPIO_PIN_RESET);


extern t_CAR car;


void Set_Motor(int16_t Motor_A,int16_t Motor_B);
void Get_All_Speed(t_CAR* m_car,int16_t time_IT);
void pid_Init(t_CAR* m_car);
void PID(t_CAR* m_car);

#endif