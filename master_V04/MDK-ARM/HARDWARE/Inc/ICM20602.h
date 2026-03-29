#include "spi.h"

#define MPU6500_DEG_PER_LSB_2000 (float)((2 * 2000.0) / 65536.0)
	
#define ICM20602_G_PER_LSB_16     (float)((2 * 16) / 65536.0)
	

uint8_t ICM20602_Init(void);
void ICM20602_ReadRegs(uint8_t start_reg, uint8_t *buf, uint16_t len);
