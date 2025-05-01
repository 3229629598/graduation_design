#ifndef _UTIL_H
#define _UTIL_H

#define LIMIT_MIN_MAX(x,min,max) (x) = (((x)<=(min))?(min):(((x)>=(max))?(max):(x)))

static inline float CLAMP(float input, float max){
  if (input > max) return (max);
  else if (input < -max) return -(max);
  else return input;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//static float invSqrt(float x) {
//	float halfx = 0.5f * x;
//	float y = x;
//	long i = *(long*)&y;
//	i = 0x5F375A86 - (i>>1);
//	y = *(float*)&i;
//	y = y * (1.5f - (halfx * y * y));
//	return y;
//}
static float fast_inv_sqrt(float number){
	const float x2 = number * 0.5F;
	const float threehalfs = 1.5F;
	
	union {
		float    f;
		unsigned int i;
	} conv = { .f = number };
	conv.i  = 0x5F375A86 - ( conv.i >> 1 );
	conv.f *= (threehalfs - ( x2 * conv.f * conv.f ));
	conv.f *= (threehalfs - ( x2 * conv.f * conv.f ));
	conv.f *= (threehalfs - ( x2 * conv.f * conv.f ));
	return conv.f;
}

#define T_ACC_CNT 100
static float s_curve(float v_max,float cnt){
	float cntcnt;
	if(cnt < 0.0f){
		cnt = -cnt;
		v_max = -v_max;
	}
	cntcnt = cnt / ((float)T_ACC_CNT);
	if(cnt < T_ACC_CNT/2.0f){
		return 2.0f*v_max*(cntcnt*cntcnt);
	}else if(cnt < T_ACC_CNT){
		cntcnt = cntcnt - 1.0f;
		return v_max*(1.0f-2.0f*(cntcnt*cntcnt));
	}else return v_max;
}

#define ACC_FILTER_MAX_CH 3
static float get_acc(float current, int ch){
	static float last_val[ACC_FILTER_MAX_CH][3];
	float result;
	if(ch >= ACC_FILTER_MAX_CH) return 0.0f;
	//refer to the Backward differentiation formula
	result = (11*current - 18*last_val[ch][2] + 9*last_val[ch][1] - 2*last_val[ch][0])/6.0f;
	last_val[ch][0] = last_val[ch][1];
	last_val[ch][1] = last_val[ch][2];
	last_val[ch][2] = current;
	return result;
}

#endif

