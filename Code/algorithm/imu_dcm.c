#include "imu_dcm.h"
#include "util.h"
/**
 * Default constant values if they are not set in the constructor
 */
#define GRAVITY_G0 9.7833f
#define INV_G0 (1/GRAVITY_G0)
#define INV_G0_2 (INV_G0*INV_G0)

//States are lowest row of rotation matrix and gyroscope x y and z biases
//(C_31, C_32, C_33, w_b1, w_b2, w_b3)
#define DEFAULT_STATE {0,0,1,0,0,0}

//estimated variance of dcm states (gyro variance per second)
#define Q_DCM2 (0.0037f*0.0037f) 
//very small number to make bias change slowly
#define Q_GYRO_BIAS2 (2.5e-3f*2.5e-3f)

#define GYRO_BIAS_MAX (1.0f*0.01745f)


//variance of calibrated accelerometer (g-component)
#define R_ACC2 (0.003f*0.003f)
//large variance for some unknown acceleration (acc = a + g)
#define R_A2 (10*10)


#define VARIANCE_MIN_LIMIT (0.0001*0.0001) //set this to a small positive number or 0 to disable the feature.
#define VARIANCE_SAFETY_INCREMENT (0.00001*0.00001) //set this to a small positive number or 0 to disable the feature.


#ifndef NULL
#ifdef __cplusplus
#define NULL 0
#else
#define NULL ((void*)0)
#endif
#endif


float linear_a[3];
float x_last[3];
float x0, x1, x2, x3, x4, x5;
float fr0, fr1, fr2, sr0, sr1, sr2;

float P00, P01, P02, P03, P04, P05;
float P11, P12, P13, P14, P15;
float P22, P23, P24, P25;
float P33, P34, P35;
float P44, P45;
float P55;

//dcm_init_t default_init_para = {
// 	.state = NULL,
// 	.covariance = NULL,
// 	.init_dcm_variance = DEFAULT_Q_DCM2_INIT,
// 	.init_bias_variance = DEFAULT_Q_GYRO_BIAS2_INIT
//};


void init_imu_dcm(dcm_init_t* init_para){

	float temp[] = DEFAULT_STATE;
	if (init_para->state != NULL) {
		for (int i = 0; i < 6; ++i) {
			temp[i] = init_para->state[i];
		}
	}
	x0 = temp[0];
	x1 = temp[1];
	x2 = temp[2];
	x3 = temp[3];
	x4 = temp[4];
	x5 = temp[5];

	if (init_para->covariance == NULL) {
		P00 = init_para->init_dcm_variance; P01 = 0; P02 = 0; P03 = 0; P04 = 0; P05 = 0;
		P11 = init_para->init_dcm_variance; P12 = 0; P13 = 0; P14 = 0; P15 = 0;
		P22 = init_para->init_dcm_variance; P23 = 0; P24 = 0; P25 = 0;
		P33 = init_para->init_bias_variance; P34 = 0; P35 = 0; 
		P44 = init_para->init_bias_variance; P45 = 0;
		P55 = init_para->init_bias_variance;
	}
	else {
		P00 = init_para->covariance[0];
		P01 = init_para->covariance[1];
		P02 = init_para->covariance[2];
		P03 = init_para->covariance[3];
		P04 = init_para->covariance[4];
		P05 = init_para->covariance[5];
		P11 = init_para->covariance[7];
		P12 = init_para->covariance[8];
		P13 = init_para->covariance[9];
		P14 = init_para->covariance[10];
		P15 = init_para->covariance[11];
		P22 = init_para->covariance[14];
		P23 = init_para->covariance[15];
		P24 = init_para->covariance[16];
		P25 = init_para->covariance[17];
		P33 = init_para->covariance[21];
		P34 = init_para->covariance[22];
		P35 = init_para->covariance[23];
		P44 = init_para->covariance[28];
		P45 = init_para->covariance[29];
		P55 = init_para->covariance[35];
	}
	//first row for alternative rotation computation
	//default is yaw = 0 which happens when fr = [1, 0, 0]
	fr0 = 1.0f;  fr1 = 0.0f;  fr2 = 0.0f;
	sr0 = 0.0f; sr1 = 1.0f; sr2 = 0.0f;
}

void imu_dcm_get_eular(eular_t* _eular){
	// compute Euler angles (not exactly a part of the extended Kalman filter)
	// yaw integration through full rotation matrix
	
//	float cy = cosf(_eular->yaw); //old angles (last state before integration)
//	float sy = sinf(_eular->yaw);
//	float d = sqrtf(x_last[1]*x_last[1] + x_last[2]*x_last[2]);
//	float d_inv = 1.0f / d;

//	// compute needed parts of rotation matrix R (state and angle based version, equivalent with the commented version above)
//	float R11 = cy * d;
//	float R12 = -(x_last[2]*sy + x_last[0]*x_last[1]*cy) * d_inv;
//	float R13 = (x_last[1]*sy - x_last[0]*x_last[2]*cy) * d_inv;
//	float R21 = sy * d;
//	float R22 = (x_last[2]*cy - x_last[0]*x_last[1]*sy) * d_inv;
//	float R23 = -(x_last[1]*cy + x_last[0]*x_last[2]*sy) * d_inv;

//	// update needed parts of R for yaw computation
//	float R11_new = R11 + dt*(u_nb2*R12 - u_nb1*R13);
//	float R21_new = R21 + dt*(u_nb2*R22 - u_nb1*R23);
//	_eular->yaw = atan2f(R21_new,R11_new);

	//-----------------------------------------------------------------------------------------------------------------------------
	//alternative method estimating the whole rotation matrix
	//integrate full rotation matrix (using first row estimate in memory)

	// normalize the second row
	float invlen = fast_inv_sqrt(sr0*sr0 + sr1*sr1 + sr2*sr2);
	sr0 *= invlen;
	sr1 *= invlen;
	sr2 *= invlen;

	// recompute the first row (ensure perpendicularity)
	fr0 = sr1*x_last[2] - sr2*x_last[1];
	fr1 = -sr0*x_last[2] + sr2*x_last[0];
	fr2 = sr0*x_last[1] - sr1*x_last[0];

	// normalize the first row
	invlen = fast_inv_sqrt(fr0*fr0 + fr1*fr1 + fr2*fr2);
	fr0 *= invlen;
	fr1 *= invlen;
	fr2 *= invlen;

	// calculate yaw from first and second row
	_eular->yaw = atan2f(sr0,fr0)*HALF_MAX_ANGLE/M_PI;
	// compute new pitch and roll angles from a posteriori states
	_eular->pitch = asinf(-x0)*HALF_MAX_ANGLE/M_PI;
	_eular->roll = atan2f(x1,x2)*HALF_MAX_ANGLE/M_PI;
}


void imu_dcm_update(imu_data_fp_t* _imu, imu_gyro_cal_t* gb,float dt){
	
	// save last state to memory for rotation estimation
	x_last[0] = x0;
	x_last[1] = x1;
	x_last[2] = x2;
	
	// control input (gyroscopes)
	float u0 = 0.0f, u1 = 0.0f, u2 = 0.0f;
	
	float recipNorm = fast_inv_sqrt(_imu->ax * _imu->ax + _imu->ay * _imu->ay + _imu->az * _imu->az);
	
	#if AUTO_STATIC_DETECTTION == 1
	/*steady state: find gyro bias*/
	static unsigned short static_state_cnt = 0;
	if(((GRAVITY_G0*recipNorm) < (1+STEADY_ACCEL_RANGE))
		&& ((GRAVITY_G0*recipNorm) > (1-STEADY_ACCEL_RANGE))
		&& fabsf(_imu->gx) < STEADY_GYRO_RANGE
		&& fabsf(_imu->gy) < STEADY_GYRO_RANGE
		&& fabsf(_imu->gz) < STEADY_GYRO_RANGE){
			
			if(static_state_cnt < STEADY_CNT_MAX){
				static_state_cnt++;
			}else{
				gb->gx_bias += BIAS_ALPHA * (_imu->gx);
				gb->gx_bias = CLAMP(gb->gx_bias,GYRO_BIAS_MAX_RAW);
				
				gb->gy_bias += BIAS_ALPHA * (_imu->gy);
				gb->gy_bias = CLAMP(gb->gy_bias,GYRO_BIAS_MAX_RAW);
					
				gb->gz_bias += BIAS_ALPHA * (_imu->gz);
				gb->gz_bias = CLAMP(gb->gz_bias,GYRO_BIAS_MAX_RAW);
		//		_imu->gx = 0.0f;
		//		_imu->gy = 0.0f;
		//		_imu->gz = 0.0f;
			}
	}else static_state_cnt = 0;
		
	#endif
	u0 = _imu->gx; u1 = _imu->gy; u2 = _imu->gz;

	
	// state prediction
	float x_0 = x0-dt*(u1*x2-u2*x1+x1*x5-x2*x4);
	float x_1 = x1+dt*(u0*x2-u2*x0+x0*x5-x2*x3);
	float x_2 = x2-dt*(u0*x1-u1*x0+x0*x4-x1*x3);
	#if AUTO_STATIC_DETECTTION == 0
	float x_3 = x3;
	float x_4 = x4;
	float x_5 = x5;
	#endif

	// covariance prediction
	float dt2 = dt*dt;
	float P_00 = P00-dt*(P05*x1*2.0f-P04*x2*2.0f+P02*(u1-x4)*2.0f-P01*(u2-x5)*2.0f)-dt2*(-Q_DCM2+x2*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5))+x1*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5))+(u2-x5)*(P15*x1-P14*x2+P12*(u1-x4)-P11*(u2-x5))-(u1-x4)*(P25*x1-P24*x2+P22*(u1-x4)-P12*(u2-x5)));
	float P_01 = P01+dt*(P05*x0-P03*x2-P15*x1+P14*x2+P02*(u0-x3)-P12*(u1-x4)-P00*(u2-x5)+P11*(u2-x5))+dt2*(x2*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5))+x0*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5))+(u2-x5)*(P05*x1-P04*x2+P02*(u1-x4)-P01*(u2-x5))-(u0-x3)*(P25*x1-P24*x2+P22*(u1-x4)-P12*(u2-x5)));
	float P_02 = P02-dt*(P04*x0-P03*x1+P25*x1-P24*x2+P01*(u0-x3)-P00*(u1-x4)+P22*(u1-x4)-P12*(u2-x5))-dt2*(x1*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5))-x0*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5))+(u1-x4)*(P05*x1-P04*x2+P02*(u1-x4)-P01*(u2-x5))-(u0-x3)*(P15*x1-P14*x2+P12*(u1-x4)-P11*(u2-x5)));
	float P_03 = P03-dt*(P35*x1-P34*x2+P23*(u1-x4)-P13*(u2-x5));
	float P_04 = P04-dt*(P45*x1-P44*x2+P24*(u1-x4)-P14*(u2-x5));
	float P_05 = P05+dt*(P45*x2-P55*x1-P25*(u1-x4)+P15*(u2-x5));
	float P_11 = P11+dt*(P15*x0*2.0f-P13*x2*2.0f+P12*(u0-x3)*2.0f-P01*(u2-x5)*2.0f)-dt2*(-Q_DCM2+x2*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5))+x0*(P35*x2-P55*x0-P25*(u0-x3)+P05*(u2-x5))+(u2-x5)*(P05*x0-P03*x2+P02*(u0-x3)-P00*(u2-x5))-(u0-x3)*(P25*x0-P23*x2+P22*(u0-x3)-P02*(u2-x5)));
	float P_12 = P12-dt*(P14*x0-P13*x1-P25*x0+P23*x2+P11*(u0-x3)-P01*(u1-x4)-P22*(u0-x3)+P02*(u2-x5))+dt2*(x1*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5))-x0*(P45*x0-P34*x2+P24*(u0-x3)-P04*(u2-x5))+(u1-x4)*(P05*x0-P03*x2+P02*(u0-x3)-P00*(u2-x5))-(u0-x3)*(P15*x0-P13*x2+P12*(u0-x3)-P01*(u2-x5)));
	float P_13 = P13+dt*(P35*x0-P33*x2+P23*(u0-x3)-P03*(u2-x5));
	float P_14 = P14+dt*(P45*x0-P34*x2+P24*(u0-x3)-P04*(u2-x5));
	float P_15 = P15-dt*(P35*x2-P55*x0-P25*(u0-x3)+P05*(u2-x5));
	float P_22 = P22-dt*(P24*x0*2.0f-P23*x1*2.0f+P12*(u0-x3)*2.0f-P02*(u1-x4)*2.0f)-dt2*(-Q_DCM2+x1*(P34*x0-P33*x1+P13*(u0-x3)-P03*(u1-x4))+x0*(P34*x1-P44*x0-P14*(u0-x3)+P04*(u1-x4))+(u1-x4)*(P04*x0-P03*x1+P01*(u0-x3)-P00*(u1-x4))-(u0-x3)*(P14*x0-P13*x1+P11*(u0-x3)-P01*(u1-x4)));
	float P_23 = P23-dt*(P34*x0-P33*x1+P13*(u0-x3)-P03*(u1-x4));
	float P_24 = P24+dt*(P34*x1-P44*x0-P14*(u0-x3)+P04*(u1-x4));
	float P_25 = P25+dt*(P35*x1-P45*x0-P15*(u0-x3)+P05*(u1-x4));
	float P_33 = P33+dt2*Q_GYRO_BIAS2;
	float P_34 = P34;
	float P_35 = P35;
	float P_44 = P44+dt2*Q_GYRO_BIAS2;
	float P_45 = P45;
	float P_55 = P55+dt2*Q_GYRO_BIAS2;

	// measurements (accelerometers)
	float z0 = _imu->ax * INV_G0;
	float z1 = _imu->ay * INV_G0;
	float z2 = _imu->az * INV_G0;

	// Kalman innovation
	float y0 = z0-x_0;
	float y1 = z1-x_1;
	float y2 = z2-x_2;

	float a_len = sqrtf(y0*y0+y1*y1+y2*y2) * GRAVITY_G0;
	float r_adab = (R_ACC2 + a_len*R_A2) * INV_G0_2;

	// innovation covariance
	float S00 = P_00 + r_adab;
	float S01 = P_01;
	float S02 = P_02;
	float S11 = P_11 + r_adab;
	float S12 = P_12;
	float S22 = P_22 + r_adab;

        // verify that the innovation covariance is large enough
	if (S00 < VARIANCE_MIN_LIMIT) S00 = VARIANCE_MIN_LIMIT;
	if (S11 < VARIANCE_MIN_LIMIT) S11 = VARIANCE_MIN_LIMIT;
	if (S22 < VARIANCE_MIN_LIMIT) S22 = VARIANCE_MIN_LIMIT;

	// determinant of S
	float det_S = -S00*(S12*S12) - (S02*S02)*S11 - (S01*S01)*S22 + S01*S02*S12*2.0f + S00*S11*S22;
	if(det_S == 0.0f) return;

	// Kalman gain
	float invPart = 1.0f / det_S;
	float K00 = -(S02*(P_02*S11-P_01*S12)-S01*(P_02*S12-P_01*S22)+P_00*(S12*S12)-P_00*S11*S22)*invPart;
	float K01 = -(S12*(P_02*S00-P_00*S02)-S01*(P_02*S02-P_00*S22)+P_01*(S02*S02)-P_01*S00*S22)*invPart;
	float K02 = -(S12*(P_01*S00-P_00*S01)-S02*(P_01*S01-P_00*S11)+P_02*(S01*S01)-P_02*S00*S11)*invPart;
	float K10 = -(S02*(P_12*S11-P_11*S12)-S01*(P_12*S12-P_11*S22)+P_01*(S12*S12)-P_01*S11*S22)*invPart;
	float K11 = -(S12*(P_12*S00-P_01*S02)-S01*(P_12*S02-P_01*S22)+P_11*(S02*S02)-P_11*S00*S22)*invPart;
	float K12 = (S12*(P_01*S01-P_11*S00)+S02*(P_11*S01-P_01*S11)-P_12*(S01*S01)+P_12*S00*S11)*invPart;
	float K20 = (S02*(P_12*S12-P_22*S11)+S01*(P_22*S12-P_12*S22)-P_02*(S12*S12)+P_02*S11*S22)*invPart;
	float K21 = (S12*(P_02*S02-P_22*S00)+S01*(P_22*S02-P_02*S22)-P_12*(S02*S02)+P_12*S00*S22)*invPart;
	float K22 = (S12*(P_02*S01-P_12*S00)+S02*(P_12*S01-P_02*S11)-P_22*(S01*S01)+P_22*S00*S11)*invPart;
	float K30 = (S02*(P_13*S12-P_23*S11)+S01*(P_23*S12-P_13*S22)-P_03*(S12*S12)+P_03*S11*S22)*invPart;
	float K31 = (S12*(P_03*S02-P_23*S00)+S01*(P_23*S02-P_03*S22)-P_13*(S02*S02)+P_13*S00*S22)*invPart;
	float K32 = (S12*(P_03*S01-P_13*S00)+S02*(P_13*S01-P_03*S11)-P_23*(S01*S01)+P_23*S00*S11)*invPart;
	float K40 = (S02*(P_14*S12-P_24*S11)+S01*(P_24*S12-P_14*S22)-P_04*(S12*S12)+P_04*S11*S22)*invPart;
	float K41 = (S12*(P_04*S02-P_24*S00)+S01*(P_24*S02-P_04*S22)-P_14*(S02*S02)+P_14*S00*S22)*invPart;
	float K42 = (S12*(P_04*S01-P_14*S00)+S02*(P_14*S01-P_04*S11)-P_24*(S01*S01)+P_24*S00*S11)*invPart;
	float K50 = (S02*(P_15*S12-P_25*S11)+S01*(P_25*S12-P_15*S22)-P_05*(S12*S12)+P_05*S11*S22)*invPart;
	float K51 = (S12*(P_05*S02-P_25*S00)+S01*(P_25*S02-P_05*S22)-P_15*(S02*S02)+P_15*S00*S22)*invPart;
	float K52 = (S12*(P_05*S01-P_15*S00)+S02*(P_15*S01-P_05*S11)-P_25*(S01*S01)+P_25*S00*S11)*invPart;

	// update a posteriori
	x0 = x_0 + K00*y0 + K01*y1 + K02*y2;
	x1 = x_1 + K10*y0 + K11*y1 + K12*y2;
	x2 = x_2 + K20*y0 + K21*y1 + K22*y2;
	
	#if AUTO_STATIC_DETECTTION == 0
	x3 = x_3 + K30*y0 + K31*y1 + K32*y2;
	x3 = CLAMP(x3, GYRO_BIAS_MAX);
//	//x3 = CLAMP((x_3 + K30*y0 + K31*y1 + K32*y2), GYRO_BIAS_MAX);
	x4 = x_4 + K40*y0 + K41*y1 + K42*y2;
	x4 = CLAMP(x4, GYRO_BIAS_MAX);
//	//x4 = CLAMP((x_4 + K40*y0 + K41*y1 + K42*y2), GYRO_BIAS_MAX);
	x5 = x_5 + K50*y0 + K51*y1 + K52*y2;
	x5 = CLAMP(x5, GYRO_BIAS_MAX);
//	//x5 = CLAMP((x_5 + K50*y0 + K51*y1 + K52*y2), GYRO_BIAS_MAX);
	#else
	x3 = 0.0f; x4 = 0.0f; x5 = 0.0f;
	#endif

	// update a posteriori covariance
	float K00_1 = K00 - 1.0f;
	float K11_1 = K11 - 1.0f;
	float K22_1 = K22 - 1.0f;

	float common1 = P_01*K00_1 + K01*P_11 + K02*P_12;
	float common2 = P_02*K00_1 + K01*P_12 + K02*P_22;
	float common3 = P_00*K00_1 + K01*P_01 + K02*P_02;
	float common4 = P_01*K11_1 + K10*P_00 + K12*P_02;
	float common5 = P_12*K11_1 + K10*P_02 + K12*P_22;
	float common6 = P_11*K11_1 + K10*P_01 + K12*P_12;
	float common7 = P_02*K22_1 + K20*P_00 + K21*P_01;
	float common8 = P_12*K22_1 + K20*P_01 + K21*P_11;
	float common9 = P_22*K22_1 + K20*P_02 + K21*P_12;
	float commonA = -P_03 + K30*P_00 + K31*P_01 + K32*P_02;
	float commonB = -P_13 + K30*P_01 + K31*P_11 + K32*P_12;
	float commonC = -P_23 + K30*P_02 + K31*P_12 + K32*P_22;

	float P__00 = K01*common1+K02*common2+(K00*K00)*r_adab+(K01*K01)*r_adab+(K02*K02)*r_adab+K00_1*common3;
	float P__01 = K10*common3+K12*common2+K11_1*common1+K00*K10*r_adab+K01*K11*r_adab+K02*K12*r_adab;
	float P__02 = K20*common3+K21*common1+K22_1*common2+K00*K20*r_adab+K01*K21*r_adab+K02*K22*r_adab;
	float P__03 = -P_03*K00_1+K30*common3+K31*common1+K32*common2-K01*P_13-K02*P_23+K00*K30*r_adab+K01*K31*r_adab+K02*K32*r_adab;
	float P__04 = -P_04*K00_1+K40*common3+K41*common1+K42*common2-K01*P_14-K02*P_24+K00*K40*r_adab+K01*K41*r_adab+K02*K42*r_adab;
	float P__05 = -P_05*K00_1+K50*common3+K51*common1+K52*common2-K01*P_15-K02*P_25+K00*K50*r_adab+K01*K51*r_adab+K02*K52*r_adab;
	float P__11 = K10*common4+K12*common5+(K10*K10)*r_adab+(K11*K11)*r_adab+(K12*K12)*r_adab+K11_1*common6;
	float P__12 = K20*common4+K21*common6+K22_1*common5+K10*K20*r_adab+K11*K21*r_adab+K12*K22*r_adab;
	float P__13 = -P_13*K11_1+K30*common4+K31*common6+K32*common5-K10*P_03-K12*P_23+K10*K30*r_adab+K11*K31*r_adab+K12*K32*r_adab;
	float P__14 = -P_14*K11_1+K40*common4+K41*common6+K42*common5-K10*P_04-K12*P_24+K10*K40*r_adab+K11*K41*r_adab+K12*K42*r_adab;
	float P__15 = -P_15*K11_1+K50*common4+K51*common6+K52*common5-K10*P_05-K12*P_25+K10*K50*r_adab+K11*K51*r_adab+K12*K52*r_adab;
	float P__22 = K20*common7+K21*common8+(K20*K20)*r_adab+(K21*K21)*r_adab+(K22*K22)*r_adab+K22_1*common9;
	float P__23 = -P_23*K22_1+K30*common7+K31*common8+K32*common9-K20*P_03-K21*P_13+K20*K30*r_adab+K21*K31*r_adab+K22*K32*r_adab;
	float P__24 = -P_24*K22_1+K40*common7+K41*common8+K42*common9-K20*P_04-K21*P_14+K20*K40*r_adab+K21*K41*r_adab+K22*K42*r_adab;
	float P__25 = -P_25*K22_1+K50*common7+K51*common8+K52*common9-K20*P_05-K21*P_15+K20*K50*r_adab+K21*K51*r_adab+K22*K52*r_adab;
	float P__33 = P_33+(K30*K30)*r_adab+(K31*K31)*r_adab+(K32*K32)*r_adab+K30*commonA+K31*commonB+K32*commonC-K30*P_03-K31*P_13-K32*P_23;
	float P__34 = P_34+K40*commonA+K41*commonB+K42*commonC-K30*P_04-K31*P_14-K32*P_24+K30*K40*r_adab+K31*K41*r_adab+K32*K42*r_adab;
	float P__35 = P_35+K50*commonA+K51*commonB+K52*commonC-K30*P_05-K31*P_15-K32*P_25+K30*K50*r_adab+K31*K51*r_adab+K32*K52*r_adab;
	float P__44 = P_44+(K40*K40)*r_adab+(K41*K41)*r_adab+(K42*K42)*r_adab+K40*(-P_04+K40*P_00+K41*P_01+K42*P_02)+K41*(-P_14+K40*P_01+K41*P_11+K42*P_12)+K42*(-P_24+K40*P_02+K41*P_12+K42*P_22)-K40*P_04-K41*P_14-K42*P_24;
	float P__45 = P_45+K50*(-P_04+K40*P_00+K41*P_01+K42*P_02)+K51*(-P_14+K40*P_01+K41*P_11+K42*P_12)+K52*(-P_24+K40*P_02+K41*P_12+K42*P_22)-K40*P_05-K41*P_15-K42*P_25+K40*K50*r_adab+K41*K51*r_adab+K42*K52*r_adab;
	float P__55 = P_55+(K50*K50)*r_adab+(K51*K51)*r_adab+(K52*K52)*r_adab+K50*(-P_05+K50*P_00+K51*P_01+K52*P_02)+K51*(-P_15+K50*P_01+K51*P_11+K52*P_12)+K52*(-P_25+K50*P_02+K51*P_12+K52*P_22)-K50*P_05-K51*P_15-K52*P_25;

	// Normalization of covariance
	float inv_len = fast_inv_sqrt(x0*x0 + x1*x1 + x2*x2);
	float invlen3 = inv_len*inv_len*inv_len;
	float invlen32 = (invlen3*invlen3);

	float x1x1_x2x2 = (x1*x1 + x2*x2);
	float x0x0_x2x2 = (x0*x0 + x2*x2);
	float x0x0_x1x1 = (x0*x0 + x1*x1);

	P00 = invlen32*(-x1x1_x2x2*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x0*x1*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2)+x0*x2*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2));
	P01 = invlen32*(-x0x0_x2x2*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2)+x0*x1*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x1*x2*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2));
	P02 = invlen32*(-x0x0_x1x1*(-P__02*x1x1_x2x2+P__12*x0*x1+P__22*x0*x2)+x0*x2*(-P__00*x1x1_x2x2+P__01*x0*x1+P__02*x0*x2)+x1*x2*(-P__01*x1x1_x2x2+P__11*x0*x1+P__12*x0*x2));
	P03 = -invlen3*(-P__03*x1x1_x2x2+P__13*x0*x1+P__23*x0*x2);
	P04 = -invlen3*(-P__04*x1x1_x2x2+P__14*x0*x1+P__24*x0*x2);
	P05 = -invlen3*(-P__05*x1x1_x2x2+P__15*x0*x1+P__25*x0*x2);
	P11 = invlen32*(-x0x0_x2x2*(-P__11*x0x0_x2x2+P__01*x0*x1+P__12*x1*x2)+x0*x1*(-P__01*x0x0_x2x2+P__00*x0*x1+P__02*x1*x2)+x1*x2*(-P__12*x0x0_x2x2+P__02*x0*x1+P__22*x1*x2));
	P12 = invlen32*(-x0x0_x1x1*(-P__12*x0x0_x2x2+P__02*x0*x1+P__22*x1*x2)+x0*x2*(-P__01*x0x0_x2x2+P__00*x0*x1+P__02*x1*x2)+x1*x2*(-P__11*x0x0_x2x2+P__01*x0*x1+P__12*x1*x2));
	P13 = -invlen3*(-P__13*x0x0_x2x2+P__03*x0*x1+P__23*x1*x2);
	P14 = -invlen3*(-P__14*x0x0_x2x2+P__04*x0*x1+P__24*x1*x2);
	P15 = -invlen3*(-P__15*x0x0_x2x2+P__05*x0*x1+P__25*x1*x2);
	P22 = invlen32*(-x0x0_x1x1*(-P__22*x0x0_x1x1+P__02*x0*x2+P__12*x1*x2)+x0*x2*(-P__02*x0x0_x1x1+P__00*x0*x2+P__01*x1*x2)+x1*x2*(-P__12*x0x0_x1x1+P__01*x0*x2+P__11*x1*x2));
	P23 = -invlen3*(-P__23*x0x0_x1x1+P__03*x0*x2+P__13*x1*x2);
	P24 = -invlen3*(-P__24*x0x0_x1x1+P__04*x0*x2+P__14*x1*x2);
	P25 = -invlen3*(-P__25*x0x0_x1x1+P__05*x0*x2+P__15*x1*x2);
	P33 = P__33;
	P34 = P__34;
	P35 = P__35;
	P44 = P__44;
	P45 = P__45;
	P55 = P__55;

    // increment covariance slightly at each iteration (nonoptimal but keeps the filter stable against rounding errors in 32bit float computation)
	P00 += VARIANCE_SAFETY_INCREMENT;
	P11 += VARIANCE_SAFETY_INCREMENT;
	P22 += VARIANCE_SAFETY_INCREMENT;
//	P33 += VARIANCE_SAFETY_INCREMENT;
//	P44 += VARIANCE_SAFETY_INCREMENT;
//	P55 += VARIANCE_SAFETY_INCREMENT;

	// variance is required to be always at least the minimum value
	if (P00 < VARIANCE_MIN_LIMIT) P00 = VARIANCE_MIN_LIMIT;
	if (P11 < VARIANCE_MIN_LIMIT) P11 = VARIANCE_MIN_LIMIT;
	if (P22 < VARIANCE_MIN_LIMIT) P22 = VARIANCE_MIN_LIMIT;
	if (P33 < VARIANCE_MIN_LIMIT) P33 = VARIANCE_MIN_LIMIT;
	if (P44 < VARIANCE_MIN_LIMIT) P44 = VARIANCE_MIN_LIMIT;
	if (P55 < VARIANCE_MIN_LIMIT) P55 = VARIANCE_MIN_LIMIT;

	// normalized a posteriori state
	x0 = x0*inv_len;
	x1 = x1*inv_len;
	x2 = x2*inv_len;
	
	float u_nb0 = u0 - x3;
	float u_nb1 = u1 - x4;
	float u_nb2 = u2 - x5;
	
	// calculate the second row (sr) from a rotated first row (rotation with bias corrected gyroscope measurement)
	sr0 = -fr1*x_last[2]+fr2*x_last[1]-dt*(x_last[1]*(fr1*u_nb0-fr0*u_nb1)+x_last[2]*(fr2*u_nb0-fr0*u_nb2));
	sr1 = fr0*x_last[2]-fr2*x_last[0]+dt*(x_last[0]*(fr1*u_nb0-fr0*u_nb1)-x_last[2]*(fr2*u_nb1-fr1*u_nb2));
	sr2 = -fr0*x_last[1]+fr1*x_last[0]+dt*(x_last[0]*(fr2*u_nb0-fr0*u_nb2)+x_last[1]*(fr2*u_nb1-fr1*u_nb2));

	// save the estimated non-gravitational acceleration
	linear_a[0] = (z0-x0)*GRAVITY_G0;
	linear_a[1] = (z1-x1)*GRAVITY_G0;
	linear_a[2] = (z2-x2)*GRAVITY_G0;
}


void get_linear_acc(float l_acc[3]){
	l_acc[0] = linear_a[0];
	l_acc[1] = linear_a[1];
	l_acc[2] = linear_a[2];
}
