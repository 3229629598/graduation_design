#include "filter.h"

//------------------------------------------
//mean filter parameter
const float m2_cof[3]={0.2,0.5,0.3};

//------------------------------------------
//IIR filter parameter (2 or 3 refer to the order)

//IIR 2-Order Filter Fs=500Hz,Fc=30Hz Butterworth
const float NUM_2[3] = {0.02785976604,  0.05571953207,  0.02785976604};
const float DEN_2[3] = {1,   -1.475480437,   0.5869194865};

// //IIR 3-Order Filter Fs=500Hz,Fc=20Hz Butterworth
const float NUM_3[4] = {0.001567010302, 0.004701030906, 0.004701030906, 0.001567010302};
const float DEN_3[4] = {1,   -2.498608351,    2.115254164,  -0.6041097045};

//IIR 3-Order Filter Fs=300Hz,Fc=90Hz Chebyshev Type II
// const float NUM_3[4] = {0.009448255412,0.01762608625,0.01762608625, 0.009448255412};
// const float DEN_3[4] = {1,-2.147717714,1.622931719,-0.4210654497};

////FIR 5-Order Filter Fs=500Hz,Fc=20Hz, Windows(Chebyshev)
//const float NUM_5[6] = {0.04964470491,0.1659367979,0.2844184935,0.2844184935,0.1659367979,0.04964470491};
const float NUM_5[6] = {0.09592749923,   0.1705435663,    0.233528927,    0.233528927,   0.1705435663,0.09592749923};
/**
  * @brief  3-order mean filter(average)
  * @param[in]  x_i input in fp32
  * @return output fp32
  */
float mean_filter_2(float x_i){
	static float x_mean[2];
	float y = m2_cof[0]*x_i + m2_cof[1]*x_mean[0] + m2_cof[2]*x_mean[1];
	x_mean[1] = x_mean[0]; x_mean[0] = x_i;
	return y;
}

/**
  * @brief  2-order IIR filter
  * @param[in]  x    input in fp32
  * @param[in]  ch   input channel, there are MAX_FILTER_CH_2 channels available
  * @return output fp32
  */
float iir_filter_2(float x, unsigned int ch){
	static float y2,x2_n1[MAX_FILTER_CH_2],x2_n2[MAX_FILTER_CH_2]
					,y2_n1[MAX_FILTER_CH_2],y2_n2[MAX_FILTER_CH_2];
	y2 = NUM_2[0]*x + NUM_2[1]*x2_n1[ch] + NUM_2[2]*x2_n2[ch]
						- DEN_2[1]*y2_n1[ch] - DEN_2[2]*y2_n2[ch];
	//if(y<EPS && y>-EPS) y = 0.0;
	y2_n2[ch]=y2_n1[ch]; y2_n1[ch]=y2;
	x2_n2[ch]=x2_n1[ch]; x2_n1[ch]=x;
	return y2;
}

/**
  * @brief  3-order IIR filter
  * @param[in]  x    input in fp32
  * @param[in]  ch   input channel, there are MAX_FILTER_CH_2 channels available
  * @return output fp32
  */

float iir_filter_3(float x, unsigned int ch){
	static float y3,x3_n1[MAX_FILTER_CH_3],x3_n2[MAX_FILTER_CH_3],x3_n3[MAX_FILTER_CH_3]
					,y3_n1[MAX_FILTER_CH_3],y3_n2[MAX_FILTER_CH_3],y3_n3[MAX_FILTER_CH_3];
	y3 = NUM_3[0]*x + NUM_3[1]*x3_n1[ch] + NUM_3[2]*x3_n2[ch] + NUM_3[3]*x3_n3[ch]
						- DEN_3[1]*y3_n1[ch] - DEN_3[2]*y3_n2[ch] - DEN_3[3]*y3_n3[ch];
	//if(y<EPS && y>-EPS) y = 0.0;
	y3_n3[ch]=y3_n2[ch]; y3_n2[ch]=y3_n1[ch]; y3_n1[ch]=y3;
	x3_n3[ch]=x3_n2[ch]; x3_n2[ch]=x3_n1[ch]; x3_n1[ch]=x;
	return y3;
}

/**
  * @brief  5-order FIR filter
  * @param[in]  x    input in fp32
  * @param[in]  ch   input channel, there are MAX_FILTER_CH_5 channels available
  * @return output fp32
  */
float fir_filter_5(float x, unsigned int ch){
	static float y5,x5_n1[MAX_FILTER_CH_5],x5_n2[MAX_FILTER_CH_5],x5_n3[MAX_FILTER_CH_5]
				,x5_n4[MAX_FILTER_CH_5],x5_n5[MAX_FILTER_CH_5];
	y5  = NUM_5[0]*x + NUM_5[1]*x5_n1[ch] + NUM_5[2]*x5_n2[ch] + NUM_5[3]*x5_n3[ch]
		+NUM_5[4]*x5_n4[ch] + NUM_5[5]*x5_n5[ch];

	x5_n5[ch]=x5_n4[ch]; x5_n4[ch]=x5_n3[ch];
	x5_n3[ch]=x5_n2[ch]; x5_n2[ch]=x5_n1[ch]; x5_n1[ch]=x;
	return y5;
}

/*
float adaptive_lp_filter(float x, float deadband, float cnt_limit)
{
    static float trust= 0.2;
    static int cmp, cmp_last, cnt = 0;
    static float x_last, x_filtered;
    cmp = (x - x_last > 0);
    if (cmp == cmp_last){
        if (abs (x - x_last) > deadband)
        	cnt += 5;
        if (cnt >= cnt_limit)
            trust += 0.1;
    }else{
        cnt = 0;
        trust  = 0.2;
        cmp_last = cmp; 
    } 
    if (trust  > 0.99) trust= 0.99;
    x_filtered = (1-trust) * x_last + trust * x;  
    x_last = x;
    return x_filtered;
} */

