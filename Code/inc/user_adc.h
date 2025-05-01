#ifndef user_adc_h
#define user_adc_h

#include "config.h"
#include "adc.h"

extern float voltage[5];

void init_vrefint_reciprocal(void);
float get_battery_voltage(void);
void user_adc_init(void);
void user_adc_loop(void);

#endif
