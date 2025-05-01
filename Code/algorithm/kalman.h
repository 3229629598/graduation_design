#ifndef _KALMAN_H
#define _KALMAN_H

typedef struct{
	float Q_cur, Q_bias, R_measure;
	float P[2][2];
	float K[2];
	float S, y, bias;
	float rate, result;
}kalman_filter_t;

/* 1 Dimension */
typedef struct {
    float x;  /* state */
    float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
    float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
    float q;  /* process(predict) noise convariance */
    float r;  /* measure noise convariance */
    float p;  /* estimated error convariance */
    float gain;
} kalman1_state;

void kalman_init(kalman_filter_t *filter, float q_cur, float q_rate, float r);
float kalman_update(kalman_filter_t *filter, float cur, float rate, float dt);
void kalman_set(kalman_filter_t *filter, float cur);

float kalman1_filter(kalman1_state *state, float z_measure);
void kalman1_init(kalman1_state *state, float init_x, float q, float r);

#endif

