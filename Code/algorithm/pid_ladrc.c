#include "pid_ladrc.h"

#include <math.h>

static inline float ladrc_sign(float val){
	if(val >= 0.0f)
		return 1.0f;
	else
		return -1.0f;
}

float fast_sqrtf(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = x* y * (1.5f - (halfx * y * y));
	
	return y;
}

float ladrc_fhan(float v1, float v2, float r0, float h0){
	float d = h0 * h0 * r0;
	float a0 = h0 * v2;
	float y = v1 + a0;
	float a1 = fast_sqrtf(d*(d + 8.0f*fabsf(y)));
	float a2 = a0 + ladrc_sign(y)*(a1-d)*0.5f;
	float sy = (ladrc_sign(y+d) - ladrc_sign(y-d))*0.5f;
	float a = (a0 + y - a2)*sy + a2;
	float sa = (ladrc_sign(a+d) - ladrc_sign(a-d))*0.5f;
	
	return -r0*(a/d - ladrc_sign(a))*sa - r0*ladrc_sign(a);
}

void ladrc_td_init(td_para_t* td, float h, float r0, float h0){
	td->h = h;
	td->r0 = r0;
	td->h0 = h0;
	//td->v1 = td->v2 = 0.0f;
}

void ladrc_td(td_para_t* td, float v){
	float fv = ladrc_fhan(td->v1 - v, td->v2, td->r0, td->h0);
	td->v1 += td->h * td->v2;
	td->v2 += td->h * fv;
}

void ladrc_init(ladrc_para_t* ladrc_para, td_para_t* td, leso_para_t* leso, float kp,float kd){
	ladrc_para->kp = kp;
	ladrc_para->kd = kd;
	//leso->a0 = 0.0f;
	ladrc_para->leso_para = leso;
	ladrc_para->td_para = td;
	//ladrc_para->u = 0.0f;
}

float ladrc(ladrc_para_t* ladrc_para ,float v){
	ladrc_td(ladrc_para->td_para, v);
	update_leso(ladrc_para->leso_para, v, ladrc_para->u);
	float e1 = ladrc_para->td_para->v1 - ladrc_para->leso_para->z1;
	float e2 = ladrc_para->td_para->v2 - ladrc_para->leso_para->z2;

	float u0 = ladrc_para->kp * e1 + ladrc_para->kd * e2;

	ladrc_para->u = u0 - ladrc_para->leso_para->z3 / ladrc_para->leso_para->b1;
	return ladrc_para->u;
}

