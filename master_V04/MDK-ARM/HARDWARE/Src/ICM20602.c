#include "ICM20602.h"

#define WHO_AM_I_ICM20602   0x12

uint8_t ICM20602_Init(void)
{
    uint8_t id;

    HAL_Delay(100);

    /* 复位设备 */
    ICM20602_WriteReg(0x6B, 0x80);
    HAL_Delay(50);

    /* 选择时钟源：PLL with X axis gyro */
    ICM20602_WriteReg(0x6B, 0x01);
    HAL_Delay(10);

    /* 关闭 FIFO */
    ICM20602_WriteReg(0x23, 0x00);

    /* 关闭 DMP */
    ICM20602_WriteReg(0x6A, 0x00);

    /* Gyro DLPF = 92Hz, ODR = 1kHz */
    ICM20602_WriteReg(0x1A, 0x02);

    /* Gyro ±2000 dps */
    ICM20602_WriteReg(0x1B, 0x10);

    /* Accel ±8g */
    ICM20602_WriteReg(0x1C, 0x10);

    /* Accel DLPF = 99Hz */
    ICM20602_WriteReg(0x1D, 0x02);

    /* 采样率分频 = 0 → 1kHz */
    ICM20602_WriteReg(0x19, 0x00);

    /* 开起中断 */
    ICM20602_WriteReg(0x38, 0x01);

    ICM20602_WriteReg(0x37, 0x10);

    /* SPI 模式 */
    ICM20602_WriteReg(0x70, 0x00);

    /* WHO_AM_I */
    id = ICM20602_ReadReg(0x75);
    if (id != WHO_AM_I_ICM20602)
    {
        return 0;
    }

    return 1;
}

void ICM20602_ReadRegs(uint8_t start_reg, uint8_t *buf, uint16_t len)
{
    uint8_t tx = start_reg | 0x80;
    ICM_CS_LOW();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 10);
    HAL_SPI_Receive(&hspi1, buf, len, 10);
    ICM_CS_HIGH();

}
