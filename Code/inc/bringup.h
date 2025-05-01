#ifndef bringup_h
#define bringup_h

#include "config.h"
#include "tim.h"
#include "user_adc.h"
#include "ne555.h"
#include "mpu6050.h"
#include "hc06.h"
#include "nrf24l01.h"
#include "data_process.h"

void bringup_init(void);
void tim5_loop(void);
void main_loop(void);

#endif
