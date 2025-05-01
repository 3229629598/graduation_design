#include "bringup.h"

void bringup_init(void) //总初始化函数
{
	HAL_Delay(100);
	user_adc_init(); //初始化光电转换
	ne555_init(); //初始化电阻测量
	mpu_init(); //初始化IMU
//	nrf_init(); //初始化无线通信
	data_init();
	HAL_Delay(100);
  HAL_TIM_Base_Start_IT(&htim5); //启动50Hz定时器
}

void tim5_loop(void) //50Hz循环函数
{
	user_adc_loop(); //将光电转换加入循环
	ne555_loop(); //将电阻测量加入循环
	mpu_loop(); //将IMU监测加入循环
	data_loop();
}

void main_loop(void) //在主函数中的循环函数
{
//	nrf_main_loop(); //将无线通信加入循环
}

