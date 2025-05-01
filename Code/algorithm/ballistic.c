#include <math.h>
#include "util.h"
#include "cordic.h"
#include "ballistic.h"

#define M_PI 3.1415926536897932384626f


void projectile_solve(float v0, float x0, float y0, ballistic_sol_t* solution){
	float x0x0 = x0*x0;
	float distance2 = x0x0+y0*y0;
	float distance = BALLISTIC_SQRT(distance2);

	if(v0 == 0.0f || (x0 == 0.0f && y0 == 0.0f)){
		solution->solution_num = 0;
		return;
	}
	
	float a = y0 + (GRAVITY_G*x0x0/(v0*v0));
	if(a > distance) {
		solution->solution_num = 0;
		return;
	}
	float alpha = BALLISTIC_ASIN(a / distance);
	float phi = M_PI/2;
	if(x0 != 0.0f) phi = BALLISTIC_ATAN(y0/x0);

	solution->solution_num = 2;
	solution->ang_solution1 = (alpha+phi)/2;
	solution->ang_solution2 = (M_PI-alpha+phi)/2;
	return;
}

void projectile_solve_optim(float v0, float x0, float y0, ballistic_sol_t* solution){
	float x0x0 = x0*x0;
	float distance2 = x0x0+y0*y0;
	float alpha,distance;// = BALLISTIC_SQRT(distance2);

	if(v0 == 0.0f || (x0 == 0.0f && y0 == 0.0f)){
		solution->solution_num = 0;
		return;
	}
	float phi = M_PI/2;
	//if(x0 != 0.0f) phi = BALLISTIC_ATAN(y0/x0);
	if(x0 != 0.0f) cordic_atan_sqrtf(y0,x0,&phi,&distance);

	float a = y0 + (GRAVITY_G*x0x0/(v0*v0));
	if(a > distance) {
		solution->solution_num = 0;
		return;
	}
	//float alpha = BALLISTIC_ASIN(a / distance);
	//float alpha = BALLISTIC_ATAN(a * fast_inv_sqrt(distance2 - a*a));
	cordic_atan_sqrtf(a * fast_inv_sqrt(distance2 - a*a),1,&alpha,0);

	solution->solution_num = 2;
	solution->ang_solution1 = (alpha+phi)/2;
	solution->ang_solution2 = (M_PI-alpha+phi)/2;
	return;
}

void ballistic_solve(float v0, float x0, float y0,ballistic_sol_t* solution){
	float v0v0 = v0*v0;
	float x0x0 = x0*x0;
	float t0_min = x0/v0;

	float poly_ln, poly_sqrt;
	float ft0,dft0,t0;

	t0 = x0/(v0*BALLISTIC_COS(solution->ang_solution1));
	if(t0 < t0_min) t0 = t0_min;

	for(int i=0;i < MAX_ITERATION_NUM;i++){
		poly_ln = 1 - K_AIR*t0;
		if(poly_ln > 0 && t0 > t0_min){
			poly_sqrt = BALLISTIC_SQRT(v0v0*t0*t0 - x0x0);
			if(solution->ang_solution1 <= 0) poly_sqrt = -poly_sqrt;
			//poly_sqrt = sqrtf(v0v0*t0*t0 - x0x0);
			ft0 = GRAVITY_G/K_AIR*(BALLISTIC_LN(poly_ln)/K_AIR + t0) + poly_sqrt - y0;
			if(ft0 < EPSILON_MAX && ft0 > -EPSILON_MAX){
				solution->solution_num = 1;
				//solution->ang_solution1 = M_PI/2 - BALLISTIC_ASIN(t0_min/t0);
				solution->ang_solution1 = BALLISTIC_ATAN(poly_sqrt/x0);
				return;
			}
			dft0 = (v0v0/poly_sqrt - GRAVITY_G/poly_ln)*t0;	
			t0 = t0 - ft0/dft0;	
		}else{
			solution->solution_num = 0;
			break;
		}
	}
	solution->solution_num = 0;
	return;
}

void ballistic_solve_optim(float v0, float x0, float y0,ballistic_sol_t* solution){
	float v0v0 = v0*v0;
	float x0x0 = x0*x0;
	float t0_min = x0/v0;

	float poly_ln,poly_sqrt,ln_val,ln_series;
	float ft0,dft0,dt0,t0;

//	float c;
//	cordic_sin_cosf(solution->ang_solution1,0,&c);
//	t0 = x0/(v0*c);
	t0 = x0/(v0*BALLISTIC_COS(solution->ang_solution1));
	if(t0 < t0_min) t0 = t0_min;

	poly_ln = 1 - K_AIR*t0;
	if(poly_ln <= 0) return;
	ln_val = BALLISTIC_LN(poly_ln)/K_AIR;
	//ln_val = cordic_logf(poly_ln)/K_AIR;

	for(int i=0;i < MAX_ITERATION_NUM;i++){
		if(t0 < 1/K_AIR && t0 > t0_min){
			poly_sqrt =  v0v0*t0*t0 - x0x0;
			poly_sqrt = BALLISTIC_SQRT_FAST(poly_sqrt);
			if(solution->ang_solution1 <= 0) poly_sqrt = -poly_sqrt;
			//poly_sqrt = sqrtf(v0v0*t0*t0 - x0x0);
			//ft0 = GRAVITY_G/K_AIR*(BALLISTIC_LN(poly_ln)/K_AIR + t0) + poly_sqrt - y0;
			ft0 = GRAVITY_G/K_AIR*(ln_val + t0) + poly_sqrt - y0;
			if(ft0 < EPSILON_MAX && ft0 > -EPSILON_MAX){
				solution->solution_num = 1;
				//solution->ang_solution1 = M_PI/2 - BALLISTIC_ASIN(t0_min/t0);
				//solution->ang_solution1 = BALLISTIC_ATAN(poly_sqrt/x0);
				cordic_atan_sqrtf(poly_sqrt,x0,&(solution->ang_solution1),0);
				return;
			}
			dft0 = (v0v0/poly_sqrt - GRAVITY_G/poly_ln)*t0;	
			dt0 = ft0/dft0;
			t0 = t0 - dt0;	

			poly_ln = 1 - K_AIR*t0;
			ln_series = dt0/poly_ln;
			ln_val += (1 + (K_AIR/2+K_AIR*K_AIR/3*ln_series)*ln_series)*ln_series;
		}else{
			solution->solution_num = 0;
			break;
		}
	}
	solution->solution_num = 0;
	return;
}
//#include <stdio.h>
// int main(int argc, char const *argv[])
// {
// 	/* code */
// 	float v0 = 16.0f;
// 	float x0; float y0;
// 	ballistic_sol_t sol;

// 	// x0=5.210151;y0=-0.520066;
// 	// //projectile_solve(v0,x0,y0,&sol);
// 	// projectile_solve_optim(v0,x0,y0,&sol);
// 	// printf("sol_num=%d,ang1=%f,ang2=%f\n"
// 	// 	,sol.solution_num,sol.ang_solution1/M_PI*180,sol.ang_solution2/M_PI*180);
// 	// //ballistic_solve(v0,x0,y0,&sol);
// 	// ballistic_solve_optim(v0,x0,y0,&sol);
// 	// printf("sol_num=%d,ang1=%f,ang2=%f\n"
// 	// 	,sol.solution_num,sol.ang_solution1/M_PI*180,sol.ang_solution2/M_PI*180);

// 	//fuzzy test
// 	// srand(time(NULL));
// 	// for(int i=0;i<10000;i++){
// 	// 	x0 = 3 + (float)rand()/RAND_MAX*20;
// 	// 	y0 = -1 + (float)rand()/RAND_MAX*3;
// 	// 	//projectile_solve(v0,x0,y0,&sol);
// 	// 	//printf("x0=%f,y0=%f\n",x0,y0);
// 	// 	projectile_solve_optim(v0,x0,y0,&sol);
// 	// 	// printf("sol_num=%d,ang1=%f,ang2=%f\n"
// 	// 	// 	,sol.solution_num,sol.ang_solution1/M_PI*180,sol.ang_solution2/M_PI*180);
// 	// 	//ballistic_solve(v0,x0,y0,&sol);
// 	// 	ballistic_solve_optim(v0,x0,y0,&sol);
// 	// 	if(sol.solution_num == 0){
// 	// 		printf("x0=%f,y0=%f\n",x0,y0);
// 	// 		printf("sol_num=%d,ang1=%f,ang2=%f\n\n"
// 	// 			,sol.solution_num,sol.ang_solution1/M_PI*180,sol.ang_solution2/M_PI*180);
// 	// 	}
// 	// }
	
// 	return 0;
// }

