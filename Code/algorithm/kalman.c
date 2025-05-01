#include "kalman.h"

/*
 * @brief  Init fields of structure kalman1_state.
  * @param[in]  filter pointer to the struct of 1-dimension kalman parameters
  * @param[in]  init_x initial value of sample
  * @param[in]  q initial predict noise covariance 
  * @param[in]  r noise covariance of measurement((3σ)^2)
  * @return none
 */
void kalman1_init(kalman1_state *state, float init_x, float q, float r){
    state->x = init_x;
    state->p = 0.0f;
    state->A = 1;
    state->H = 1;
    state->q = q;//10e-6;  /* predict noise convariance */
    state->r = r;//10e-5;  /* measure error convariance */
}

/*
 * @brief  1 Dimension Kalman filter
 * @param[in]  filter pointer to the struct of 1-dimension kalman parameters
 * @param[in]  z_measure measured value
 * @return estimated values
 */
float kalman1_filter(kalman1_state *state, float z_measure){
    /* Predict */
    state->x = state->A * state->x;
    state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */

    /* Measurement */
    state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
    state->x = state->x + state->gain * (z_measure - state->H * state->x);
    state->p = (1 - state->gain * state->H) * state->p;

    return state->x;
}




/**
  * @brief  kalman filter struct init
  * @param[in]  filter pointer to the struct of kalman parameters
  * @param[in]  q_cur noise covariance of current value
  * @param[in]  q_rate noise covariance of the rate
  * @param[in]  r noise covariance of measurement((3σ)^2)
  * @return none
  */
void kalman_init(kalman_filter_t *filter, float q_cur, float q_rate, float r){
	filter->Q_cur = q_cur;
	filter->Q_bias = q_rate;
	filter->R_measure = r;

	filter->P[0][0] = 0.0f;
	filter->P[0][1] = 0.0f;
	filter->P[1][0] = 0.0f;
	filter->P[1][1] = 0.0f;
	filter->K[0] = 0.0f;
	filter->K[1] = 0.0f;
	filter->S = 0.0f;
	filter->y = 0.0f;
	filter->bias = 0.0f;
}


/**
  * @brief  kalman filter update process
  * @param[in]  filter pointer to the struct of kalman parameters
  * @param[in]  cur current value to be filtered
  * @param[in]  rate the rate of current value(cur per sec)
  * @param[in]  dt delta time in second
  * @return the filtered value
  */
float kalman_update(kalman_filter_t *filter, float cur, float rate, float dt){
	//step 1
	filter->rate = rate - filter->bias;
	filter->result += dt * filter->rate;
	//step 2: Update estimation error covariance - Project the error covariance ahead
	filter->P[0][0] += dt * (dt * filter->P[1][1] - filter->P[0][1] - filter->P[1][0] + filter->Q_cur);
	filter->P[0][1] -= dt * filter->P[1][1];
	filter->P[1][0] -= dt * filter->P[1][1];
	filter->P[1][1] += filter->Q_bias * dt;
	//Step 3
	filter->y = cur - filter->result;
	//Step 4: Calculate Kalman gain
	filter->S = filter->P[0][0] + filter->R_measure;
	
	//Step 5: Kk = Pk|k-1 * H^T * S^-1_k
	filter->K[0] = filter->P[0][0]/filter->S;
	filter->K[1] = filter->P[1][0]/filter->S;
	
	//Step 6
	filter->result+=filter->K[0] * filter->y;
	filter->bias +=filter->K[1] * filter->y;
	
	//Step 7
	filter->P[0][0] -= filter->K[0] * filter->P[0][0];
	filter->P[0][1] -= filter->K[0] * filter->P[0][1];
	filter->P[1][0] -= filter->K[1] * filter->P[0][0];
	filter->P[1][1] -= filter->K[1] * filter->P[0][1];
	
	return filter->result;
}


/**
  * @brief  kalman filter result value set
  * @param[in]  cur value to set
  * @return none
  */
void kalman_set(kalman_filter_t *filter, float cur){
	filter->result = cur;
}

