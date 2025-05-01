#ifndef hc06_h
#define hc06_h

#include "config.h"
#include "usart.h"
#include "usbd_cdc_if.h"
 
#define BUFFER_SIZE 100 //最大接收字节数
#define rx_header 0x5A
#define tx_header 0xA5

extern uint8_t rx_buffer[100];

HAL_StatusTypeDef hc06_send(uint8_t *buf);
HAL_StatusTypeDef hc06_read(uint8_t *buf,uint8_t len);
void usart1_IRQ(void);
void hc06_init(void);
void hc06_loop(void);

#endif
