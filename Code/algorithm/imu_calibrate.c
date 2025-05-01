#include "imu_calibrate.h"


#include <string.h>
#include <math.h>
#include <stdlib.h>

#define FLT_EPSILON                1.19209290E-07f

static float accel_ref[6][3];
static float accel_ref_o[6][3];

/*
 * ===== Model =====
 * accel_corr = accel_T * (accel_raw - accel_offs)
 *
 * accel_corr[3] - fully corrected acceleration vector in body frame
 * accel_T[3][3] - accelerometers transform matrix, rotation and scaling transform
 * accel_raw[3]  - raw acceleration vector
 * accel_offs[3] - acceleration offset vector
 *
 * ===== Calibration =====
 *
 * Reference vectors
 * accel_corr_ref[6][3] = [  g  0  0 ]     // nose up			ACC_PX
 *                        | -g  0  0 |     // nose down			ACC_NX
 *                        |  0  g  0 |     // left side down	ACC_PY
 *                        |  0 -g  0 |     // right side down	ACC_NY
 *                        |  0  0  g |     // on back			ACC_PZ
 *                        [  0  0 -g ]     // level				ACC_NZ
 * accel_raw_ref[6][3]
 *
 * accel_corr_ref[i] = accel_T * (accel_raw_ref[i] - accel_offs), i = 0...5
 *
 * 6 reference vectors * 3 axes = 18 equations
 * 9 (accel_T) + 3 (accel_offs) = 12 unknown constants
 *
 * Find accel_offs
 *
 * accel_offs[i] = (accel_raw_ref[i*2][i] + accel_raw_ref[i*2+1][i]) / 2
 *
 * Find accel_T
 *
 * 9 unknown constants
 * need 9 equations -> use 3 of 6 measurements -> 3 * 3 = 9 equations
 *
 * accel_corr_ref[i*2] = accel_T * (accel_raw_ref[i*2] - accel_offs), i = 0...2
 *
 * Solve separate system for each row of accel_T:
 *
 * accel_corr_ref[j*2][i] = accel_T[i] * (accel_raw_ref[j*2] - accel_offs), j = 0...2
 *
 * A * x = b
 *
 * x = [ accel_T[0][i] ]
 *     | accel_T[1][i] |
 *     [ accel_T[2][i] ]
 *
 * b = [ accel_corr_ref[0][i] ]	// One measurement per side is enough
 *     | accel_corr_ref[2][i] |
 *     [ accel_corr_ref[4][i] ]
 *
 * a[i][j] = accel_raw_ref[i][j] - accel_offs[j], i = 0;2;4, j = 0...2
 *
 * Matrix A is common for all three systems:
 * A = [ a[0][0]  a[0][1]  a[0][2] ]
 *     | a[2][0]  a[2][1]  a[2][2] |
 *     [ a[4][0]  a[4][1]  a[4][2] ]
 *
 * x = A^-1 * b
 *
 * accel_T = A^-1 * g
*/
int get_acc_calibration_val(float accel_ref[6][3], imu_acc_cal_t* cali_val);

void gyro_calibration(imu_data_raw_t* gyro_data,imu_gyro_cal_t* gyro_val){
	if((abs(gyro_data->gx) < GYRO_STEADY_RANGE) 
		&& (abs(gyro_data->gy) < GYRO_STEADY_RANGE)
		&& (abs(gyro_data->gz) < GYRO_STEADY_RANGE)){
		gyro_val->gx_bias = ((float)gyro_data->gx + gyro_val->gx_bias)/2;
		gyro_val->gy_bias = ((float)gyro_data->gy + gyro_val->gy_bias)/2;
		gyro_val->gz_bias = ((float)gyro_data->gz + gyro_val->gz_bias)/2;
	}
}

int acc_calibration(imu_data_raw_t* acc_raw,imu_acc_cal_t* cali_val,acc_cali_state_t cali_state){
	signed short acc_data[3] = {acc_raw->ax,acc_raw->ay,acc_raw->az};
	int acc_index = (int)cali_state;

	if(acc_index < ACC_PX_O){
		if((abs(acc_data[acc_index/2]) > abs(acc_data[(acc_index/2 + 1) % 3]))
			&& (abs(acc_data[acc_index/2]) > abs(acc_data[(acc_index/2 + 2) % 3]))){
			accel_ref[acc_index][0] = accel_ref[acc_index][0]*0.99f + (float)acc_data[0]*0.01f;
			accel_ref[acc_index][1] = accel_ref[acc_index][1]*0.99f + (float)acc_data[1]*0.01f;
			accel_ref[acc_index][2] = accel_ref[acc_index][2]*0.99f + (float)acc_data[2]*0.01f;
		}
	}else if(acc_index != ACC_CAL_FINAL){
		acc_index -= ACC_PX_O;
		if((abs(acc_data[acc_index/2]) > abs(acc_data[(acc_index/2 + 1) % 3]))
			&& (abs(acc_data[acc_index/2]) > abs(acc_data[(acc_index/2 + 2) % 3]))){
			accel_ref_o[acc_index][0] = accel_ref_o[acc_index][0]*0.99f + (float)acc_data[0]*0.01f;
			accel_ref_o[acc_index][1] = accel_ref_o[acc_index][1]*0.99f + (float)acc_data[1]*0.01f;
			accel_ref_o[acc_index][2] = accel_ref_o[acc_index][2]*0.99f + (float)acc_data[2]*0.01f;
		}
	}else{
		//finalize calibration process
		for(int i = 0; i < 6;i++){
			accel_ref[i][0] = (accel_ref_o[i][0] + accel_ref[i][0])/2;
			accel_ref[i][1] = (accel_ref_o[i][1] + accel_ref[i][1])/2;
			accel_ref[i][2] = (accel_ref_o[i][2] + accel_ref[i][2])/2;
		}
		if(cali_val != 0) return get_acc_calibration_val(accel_ref,cali_val);
	}
	return CALI_OK;
}

void reset_acc_cali(unsigned int index) {
	if (index < 6){
		memset(accel_ref[index], 0, sizeof(float) * 3);
		memset(accel_ref_o[index], 0, sizeof(float) * 3);
	}
}

void get_acc_cal_tmp(unsigned int index, float* tmp,float* tmp_o) {
	if (tmp != 0) memcpy(tmp, accel_ref[index], 3 * sizeof(float));
	if (tmp_o != 0) memcpy(tmp_o, accel_ref_o[index], 3 * sizeof(float));
}

void set_acc_cal_tmp(unsigned int index, float* tmp, float* tmp_o) {
	if (tmp != 0) memcpy(accel_ref[index], tmp, 3 * sizeof(float));
	if (tmp_o != 0) memcpy(accel_ref_o[index], tmp_o, 3 * sizeof(float));
}


int mat_invert3(float src[3][3], float dst[3][3]){
	float det = src[0][0] * (src[1][1] * src[2][2] - src[1][2] * src[2][1]) -
		    src[0][1] * (src[1][0] * src[2][2] - src[1][2] * src[2][0]) +
		    src[0][2] * (src[1][0] * src[2][1] - src[1][1] * src[2][0]);

	if (fabsf(det) < FLT_EPSILON) {
		return CALI_ERR;        // Singular matrix
	}
	dst[0][0] = (src[1][1] * src[2][2] - src[1][2] * src[2][1]) / det;
	dst[1][0] = (src[1][2] * src[2][0] - src[1][0] * src[2][2]) / det;
	dst[2][0] = (src[1][0] * src[2][1] - src[1][1] * src[2][0]) / det;
	dst[0][1] = (src[0][2] * src[2][1] - src[0][1] * src[2][2]) / det;
	dst[1][1] = (src[0][0] * src[2][2] - src[0][2] * src[2][0]) / det;
	dst[2][1] = (src[0][1] * src[2][0] - src[0][0] * src[2][1]) / det;
	dst[0][2] = (src[0][1] * src[1][2] - src[0][2] * src[1][1]) / det;
	dst[1][2] = (src[0][2] * src[1][0] - src[0][0] * src[1][2]) / det;
	dst[2][2] = (src[0][0] * src[1][1] - src[0][1] * src[1][0]) / det;

	return CALI_OK;
}

int get_acc_calibration_val(float accel_ref[6][3], imu_acc_cal_t* cali_val){
	/* calculate offsets */
	for (unsigned i = 0; i < 3; i++) {
		cali_val->accel_offs[i] = (accel_ref[i * 2][i] + accel_ref[i * 2 + 1][i]) / 2;
	}

	/* fill matrix A for linear equations system*/
	float mat_A[3][3];
	memset(mat_A, 0, sizeof(mat_A));

	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			float a = accel_ref[i * 2][j] - cali_val->accel_offs[j];
			mat_A[i][j] = a;
		}
	}

	/* calculate inverse matrix for A */
	float mat_A_inv[3][3];

	if (mat_invert3(mat_A, mat_A_inv) != CALI_OK) {
		return CALI_ERR;
	}

	/* copy results to accel_T */
	for (unsigned i = 0; i < 3; i++) {
		for (unsigned j = 0; j < 3; j++) {
			/* simplify matrices mult because b has only one non-zero element == g at index i */
			if(i != j && fabsf(mat_A_inv[j][i]) > MAX_CROSS_AXIS_ACC) return CALI_ERR;
			cali_val->accel_T[j][i] = mat_A_inv[j][i] * (CONST_G_VAL);
		}
	}
	return CALI_OK;
}

void init_imu_calibration(imu_acc_cal_t* acc_cali, imu_gyro_cal_t* gyro_cali){
	if(acc_cali != 0){
		memset(acc_cali,0,sizeof(imu_acc_cal_t));
		acc_cali->accel_T[0][0] = 1.0f;
		acc_cali->accel_T[1][1] = 1.0f;
		acc_cali->accel_T[2][2] = 1.0f;
	}
	if(gyro_cali != 0){
		memset(gyro_cali,0,sizeof(imu_gyro_cal_t));
	}
}	


imu_data_fp_t imu_correct(imu_data_raw_t* imu_d
		, imu_acc_cal_t* acc_cali, imu_gyro_cal_t* gyro_cali, float tempture){
	imu_data_fp_t imu_corr;
	float ax_ub = imu_d->ax - acc_cali->accel_offs[0];
	float ay_ub = imu_d->ay - acc_cali->accel_offs[1];
	float az_ub = imu_d->az - acc_cali->accel_offs[2];
	imu_corr.ax = (acc_cali->accel_T[0][0]*ax_ub + acc_cali->accel_T[0][1]*ay_ub + acc_cali->accel_T[0][2]*az_ub);
	imu_corr.ay = (acc_cali->accel_T[1][0]*ax_ub + acc_cali->accel_T[1][1]*ay_ub + acc_cali->accel_T[1][2]*az_ub);
	imu_corr.az = (acc_cali->accel_T[2][0]*ax_ub + acc_cali->accel_T[2][1]*ay_ub + acc_cali->accel_T[2][2]*az_ub);

	imu_corr.ax *= (imu_d->a_fullscale/(float)(IMU_RAW_RANGE));
	imu_corr.ay *= (imu_d->a_fullscale/(float)(IMU_RAW_RANGE));
	imu_corr.az *= (imu_d->a_fullscale/(float)(IMU_RAW_RANGE));
	
	if(gyro_cali != 0){
		imu_corr.gx = (((float)imu_d->gx) - gyro_cali->gx_bias - (gyro_cali->gx_tco_k*tempture+gyro_cali->gx_tco_b0))
			*(imu_d->g_fullscale/(float)(IMU_RAW_RANGE));
		imu_corr.gy = (((float)imu_d->gy) - gyro_cali->gy_bias - (gyro_cali->gy_tco_k*tempture+gyro_cali->gy_tco_b0))
			*(imu_d->g_fullscale/(float)(IMU_RAW_RANGE));
		imu_corr.gz = (((float)imu_d->gz) - gyro_cali->gz_bias - (gyro_cali->gz_tco_k*tempture+gyro_cali->gz_tco_b0))
			*(imu_d->g_fullscale/(float)(IMU_RAW_RANGE));
	}
	
	return imu_corr;
}



//------------------------------------------------------------------------------------------------------

