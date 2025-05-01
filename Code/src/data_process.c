#include "data_process.h"

usb_tx_bag usb_tx_data;
uint8_t tx_len;

void data_init(void)
{
	tx_len=sizeof(usb_tx_bag);
}

void data_loop(void)
{
	usb_tx_data.header=usb_tx_header;
	usb_tx_data.imu_rpy[0]=(float)mpu6050_data.Gx;
	usb_tx_data.imu_rpy[1]=(float)mpu6050_data.Gy;
	usb_tx_data.imu_rpy[2]=(float)mpu6050_data.Gz;
	memcpy((uint8_t*)usb_tx_data.light,(uint8_t*)voltage,finger*4);
	memcpy((uint8_t*)usb_tx_data.electric,(uint8_t*)r,finger*4);
	Append_CRC16_Check_Sum((uint8_t*)&usb_tx_data,tx_len);
	CDC_Transmit_FS((uint8_t*)&usb_tx_data,tx_len);
}
