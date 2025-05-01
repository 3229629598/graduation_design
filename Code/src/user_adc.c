#include "user_adc.h"

float voltage_vrefint_proportion;

static uint16_t adcx_get_chx_value(ADC_HandleTypeDef *ADCx, uint32_t ch)
{
    static ADC_ChannelConfTypeDef sConfig = {0};
    sConfig.Channel = ch;
    sConfig.Rank = 1;
    sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;

    if (HAL_ADC_ConfigChannel(ADCx, &sConfig) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_ADC_Start(ADCx);

    HAL_ADC_PollForConversion(ADCx, 10);
    return (uint16_t)HAL_ADC_GetValue(ADCx);

}

void init_vrefint_reciprocal(void)
{
    uint8_t i = 0;
    uint32_t total_adc = 0;
    for(i = 0; i < 200; i++)
    {
        total_adc += adcx_get_chx_value(&hadc1, ADC_CHANNEL_VREFINT);
    }

    voltage_vrefint_proportion = 200 * 1.2f / total_adc;
}

uint16_t adc_value[100]={0};
uint16_t adc_average[5]={0};
float voltage[5]={0};

void user_adc_init(void) //初始化函数
{
	init_vrefint_reciprocal();
	MX_ADC1_Init();
}

void user_adc_loop(void) //循环执行函数
{
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&adc_value,100);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) //ADC中断回调函数
{
	HAL_ADC_Stop_DMA(&hadc1);
	memset(adc_average,0,sizeof(adc_average));
	for(int i=0;i<5;i++)
	{
		for(int j=0;j<20;j++)
		{
			adc_average[i]+=adc_value[j*5+i];
		}
		adc_average[i]/=20;
		voltage[i]=adc_average[i]*voltage_vrefint_proportion;
//		CDC_Transmit_FS((uint8_t*)(&voltage[i]),sizeof(adc_average[i]));
	}
}

