#ifndef _BMP180_H_
#define _BMP180_H_

#include "stm32h7xx_hal.h"

void BMP180_Start (void);

float BMP180_calculate_true_temperature (void);

float BMP180_calculate_true_pressure (int oss);


#endif 
