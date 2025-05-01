#include "ne555.h"

uint16_t last_cnt[5],new_cnt[5],cnt[5];
float ln2;
float c2=0.0000001,r1=1500,r2=1500;
float fr[5],r[5];

void ne555_init(void)
{
	__HAL_TIM_CLEAR_FLAG(&htim1, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(&htim2, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(&htim3, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(&htim4, TIM_SR_UIF);
	__HAL_TIM_CLEAR_FLAG(&htim8, TIM_SR_UIF);
	HAL_TIM_Base_Start(&htim2);
	ln2=log(2);
}

void ne555_loop(void)
{
	
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	memcpy(last_cnt,new_cnt,sizeof(last_cnt));
	new_cnt[0] = __HAL_TIM_GetCounter(&htim1);
	new_cnt[1] = __HAL_TIM_GetCounter(&htim2);
	new_cnt[2] = __HAL_TIM_GetCounter(&htim3);
	new_cnt[3] = __HAL_TIM_GetCounter(&htim4);
	new_cnt[4] = __HAL_TIM_GetCounter(&htim8);
	for(int i=0;i<5;i++)
	{
		cnt[i]=new_cnt[i]-last_cnt[i];
		fr[i]=cnt[i]*tim5_f;
		r[i]=(1/(ln2*c2*fr[i])-r1)/2-r2;
	}
	
}
