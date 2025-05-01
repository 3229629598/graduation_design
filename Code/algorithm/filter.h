#ifndef _FILTER_H
#define _FILTER_H

//IIR filter channel number (2 or 3 refer to the order)

#define MAX_FILTER_CH_2 4
#define MAX_FILTER_CH_3 4
#define MAX_FILTER_CH_5 2

#define CHASSIS_FILTER3_CH1 0
#define CHASSIS_FILTER3_CH2 1
#define CHASSIS_FILTER3_CH3 2

#define GIMBAL_PITCH_FILTER2_RPM_CH 0
#define GIMBAL_YAW_FILTER2_RPM_CH 1
#define SHOOT_LEFT_FILTER2_RPM_CH 2
#define SHOOT_RIGHT_FILTER2_RPM_CH 3


float mean_filter_2(float x_i);
float iir_filter_2(float x, unsigned int ch);
float iir_filter_3(float x, unsigned int ch);

#endif

