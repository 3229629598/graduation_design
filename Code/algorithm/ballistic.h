#ifndef _BALLISTIC_H
#define _BALLISTIC_H

#define GRAVITY_G 9.7833f
/*%F=C_d/2*A*¦Ñ*v^2, C_d=0.47 */
/*K_AIR(lambda) = 2*F/m = (C_d*A*¦Ñ*v^2)/m*/
#define K_AIR 0.0199f //42mm
//#define K_AIR 0.0399f //17mm

#define MAX_ITERATION_NUM 30
#define EPSILON_MAX 1e-4f

#define BALLISTIC_COS(x) cosf(x)
#define BALLISTIC_ASIN(x) asinf(x)
#define BALLISTIC_ATAN(x) atanf(x)
#define BALLISTIC_SQRT(x) sqrtf(x)
#define BALLISTIC_SQRT_FAST(x) ((x)*fast_inv_sqrt(x))
#define BALLISTIC_LN(x) logf(x)

typedef struct{
	unsigned int solution_num;
	float ang_solution1;
	float ang_solution2;
}ballistic_sol_t;

void ballistic_solve_optim(float v0, float x0, float y0,ballistic_sol_t* solution);
void projectile_solve_optim(float v0, float x0, float y0, ballistic_sol_t* solution);
void ballistic_solve(float v0, float x0, float y0,ballistic_sol_t* solution);
void projectile_solve(float v0, float x0, float y0, ballistic_sol_t* solution);


#endif
