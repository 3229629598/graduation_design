
#ifndef _PID_LADRC_H
#define _PID_LADRC_H

#include "pid_leso.h"

typedef struct
{
	float v1,v2;
	float r0,h0,h;
}td_para_t;


typedef struct{
	leso_para_t* leso_para;
	td_para_t* td_para;
	float kp, kd;
	float u;
}ladrc_para_t;

void ladrc_td_init(td_para_t* td, float h, float r0, float h0);
void ladrc_init(ladrc_para_t* ladrc_para, td_para_t* td, leso_para_t* leso, float kp,float kd);
float ladrc(ladrc_para_t* ladrc_para ,float v);

#endif

