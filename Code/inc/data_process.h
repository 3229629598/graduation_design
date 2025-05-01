#ifndef data_process_h
#define data_process_h

#include "config.h"
#include "mpu6050.h"
#include "user_adc.h"
#include "ne555.h"
#include "crc.h"
#include "usbd_cdc_if.h"

#define usb_tx_header 0x5a
#define finger 5

typedef __packed struct
{
	uint8_t header;
  float imu_rpy[3];
	float light[5];
	float electric[5];
  uint16_t crc_sum;
}usb_tx_bag;


void data_init(void);
void data_loop(void);

#endif
