#include "spi.h"
#include <math.h>

void BMP280_GetData(float *temperature,float *pressure,float *altitude);
uint8_t BMP280_Init(void);