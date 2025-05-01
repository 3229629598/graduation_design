#include "hc06.h"

uint8_t rx_buffer[100]={0}; //�������ݻ�������
uint8_t rx_len=0; //����һ֡���ݵĳ���
uint8_t init_flag=0;
float rx_data[5],tx_data[13];

void usart1_IRQ(void)//�����жϻص�����
{
		uint32_t tmp_flag = 0;
		uint32_t temp;
		tmp_flag =__HAL_UART_GET_FLAG(&huart1,UART_FLAG_IDLE); //��ȡIDLE��־λ
		if((tmp_flag != RESET)) //idle��־����λ
		{ 
			__HAL_UART_CLEAR_IDLEFLAG(&huart1); //�����־λ
			HAL_UART_DMAStop(&huart1);
			temp = __HAL_DMA_GET_COUNTER(&hdma_usart1_rx); // ��ȡDMA��δ��������ݸ���   
			rx_len = BUFFER_SIZE - temp; //�ܼ�����ȥδ��������ݸ������õ��Ѿ����յ����ݸ���
//			if(rx_len==(sizeof(rx_data)+1) && rx_buffer[0]==rx_header)
//			{
//				memcpy(rx_data,rx_buffer,sizeof(rx_data));
//			}
//			if(rx_len==sizeof("OK")&&init_flag==0)
//			{
//				init_flag=1;
//			}
			init_flag=1;
			
		}
}

HAL_StatusTypeDef hc06_send(uint8_t *buf)
{
	return HAL_UART_Transmit_DMA(&huart1,buf,sizeof(buf));
}

HAL_StatusTypeDef hc06_read(uint8_t *buf,uint8_t len)
{
	return HAL_UART_Receive_DMA(&huart1,buf,len);
}

void hc06_init(void)
{
	__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
	HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);

//	while(1)
//	{
//		if(init_flag)
//			break;
//		else
//			hc06_send((uint8_t*)"AT");
//		HAL_Delay(50);
//	}
//	HAL_GPIO_TogglePin(LED_USER_GPIO_Port,LED_USER_Pin);
//	hc06_send((uint8_t*)"AT+ROLE=M");
//	hc06_send((uint8_t*)"AT+NAME=Hello");
	
}

void hc06_loop(void)
{
	if(init_flag)
	{
		hc06_send((uint8_t*)rx_data);
		init_flag=0;
		memcpy(rx_data,rx_buffer,sizeof(rx_data));
		rx_len=0;
		memset(rx_buffer,0,BUFFER_SIZE);
		HAL_UART_Receive_DMA(&huart1,rx_buffer,BUFFER_SIZE);
	}
}
