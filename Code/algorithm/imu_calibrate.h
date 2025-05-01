#ifndef _IMU_CALI_H
#define _IMU_CALI_H

#define CALI_ERR (-1)
#define CALI_OK (0)


typedef struct{
	float accel_T[3][3];
	float accel_offs[3];
}imu_acc_cal_t;

typedef struct{
	float gx_bias, gy_bias, gz_bias;
	float gx_tco_k,gx_tco_b0;
	float gy_tco_k,gy_tco_b0;
	float gz_tco_k,gz_tco_b0;
}imu_gyro_cal_t;

typedef enum{
	ACC_PX = 0,ACC_NX = 1, ACC_PX_O = 6,ACC_NX_O = 7,
	ACC_PY = 2,ACC_NY = 3, ACC_PY_O = 8,ACC_NY_O = 9,
	ACC_PZ = 4,ACC_NZ = 5, ACC_PZ_O = 10,ACC_NZ_O = 11,
	ACC_CAL_FINAL = 12
}acc_cali_state_t;

#include "imu_fusion.h"

#define GYRO_STEADY_RANGE (50)

#define IMU_RAW_RANGE (32768)

//#define CONST_G_VAL ((9.7833f))
#define CONST_G_VAL (IMU_RAW_RANGE/3) //raw imu value
#define MAX_CROSS_AXIS_ACC (0.01f)
#ifdef __cplusplus 

extern "C" {
#endif
void gyro_calibration(imu_data_raw_t* gyro_data,imu_gyro_cal_t* gyro_val);
int acc_calibration(imu_data_raw_t* acc_raw,imu_acc_cal_t* cali_val,acc_cali_state_t cali_state);
void init_imu_calibration(imu_acc_cal_t* acc_cali, imu_gyro_cal_t* gyro_cali);

imu_data_fp_t imu_correct(imu_data_raw_t* imu_d
	,imu_acc_cal_t* acc_cali, imu_gyro_cal_t* gyro_cali, float tempture);

void reset_acc_cali(unsigned int index);
void get_acc_cal_tmp(unsigned int index, float* tmp, float* tmp_o);
void set_acc_cal_tmp(unsigned int index, float* tmp, float* tmp_o);

#ifdef __cplusplus 
}
#endif

#endif

