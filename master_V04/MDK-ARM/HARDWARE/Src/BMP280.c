#include "BMP280.h"
#include <stdint.h>

/* BMP280 寄存器 */
#define BMP280_REG_ID        0xD0
#define BMP280_REG_RESET     0xE0
#define BMP280_REG_STATUS    0xF3
#define BMP280_REG_CTRL_MEAS 0xF4
#define BMP280_REG_CONFIG    0xF5
#define BMP280_REG_PRESS_MSB 0xF7

#define BMP280_RESET_VALUE   0xB6


#define SEA_LEVEL_PRESSURE 101325.0f  // Pa

/* 校准参数结构体 */
typedef struct
{
    uint16_t dig_T1;
    int16_t  dig_T2;
    int16_t  dig_T3;

    uint16_t dig_P1;
    int16_t  dig_P2;
    int16_t  dig_P3;
    int16_t  dig_P4;
    int16_t  dig_P5;
    int16_t  dig_P6;
    int16_t  dig_P7;
    int16_t  dig_P8;
    int16_t  dig_P9;
} bmp280_calib_t;

bmp280_calib_t bmp280_calib;
int32_t t_fine;


void BMP280_WriteReg(uint8_t reg, uint8_t val)
{
    uint8_t tx[2];
    tx[0] = reg & 0x7F;   // 写：MSB=0
    tx[1] = val;
		BMP_CS_LOW();
	//__HAL_SPI_DISABLE(&hspi2);
    HAL_SPI_Transmit(&hspi2, tx, 2, HAL_MAX_DELAY);
		BMP_CS_HIGH();
}

void BMP280_ReadRegs(uint8_t reg, uint8_t *buf, uint8_t len)
{
    uint8_t tx = reg | 0x80;  // 读：MSB=1
		BMP_CS_LOW();
	//__HAL_SPI_DISABLE(&hspi2);
    HAL_SPI_Transmit(&hspi2, &tx, 1, HAL_MAX_DELAY);
    HAL_SPI_Receive(&hspi2, buf, len, HAL_MAX_DELAY);
		BMP_CS_HIGH();
}

void BMP280_ReadCalibration(void)
{
    uint8_t buf[24];

    BMP280_ReadRegs(0x88, buf, 24);

    bmp280_calib.dig_T1 = (buf[1] << 8) | buf[0];
    bmp280_calib.dig_T2 = (buf[3] << 8) | buf[2];
    bmp280_calib.dig_T3 = (buf[5] << 8) | buf[4];

    bmp280_calib.dig_P1 = (buf[7] << 8) | buf[6];
    bmp280_calib.dig_P2 = (buf[9] << 8) | buf[8];
    bmp280_calib.dig_P3 = (buf[11] << 8) | buf[10];
    bmp280_calib.dig_P4 = (buf[13] << 8) | buf[12];
    bmp280_calib.dig_P5 = (buf[15] << 8) | buf[14];
    bmp280_calib.dig_P6 = (buf[17] << 8) | buf[16];
    bmp280_calib.dig_P7 = (buf[19] << 8) | buf[18];
    bmp280_calib.dig_P8 = (buf[21] << 8) | buf[20];
    bmp280_calib.dig_P9 = (buf[23] << 8) | buf[22];
}
void BMP280_ReadRaw(int32_t *temp, int32_t *press)
{
    uint8_t buf[6];

    BMP280_ReadRegs(BMP280_REG_PRESS_MSB, buf, 6);

    *press = (int32_t)((buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4));
    *temp  = (int32_t)((buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4));
}

int32_t BMP280_Compensate_T(int32_t adc_T)
{
    int32_t var1, var2;

    var1 = ((((adc_T >> 3) - ((int32_t)bmp280_calib.dig_T1 << 1))) *
            ((int32_t)bmp280_calib.dig_T2)) >> 11;

    var2 = (((((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1)) *
              ((adc_T >> 4) - ((int32_t)bmp280_calib.dig_T1))) >> 12) *
            ((int32_t)bmp280_calib.dig_T3)) >> 14;

    t_fine = var1 + var2;
    return (t_fine * 5 + 128) >> 8;
}
uint32_t BMP280_Compensate_P(int32_t adc_P)
{
    int64_t var1, var2, p;

    var1 = ((int64_t)t_fine) - 128000;
    var2 = var1 * var1 * bmp280_calib.dig_P6;
    var2 = var2 + ((var1 * bmp280_calib.dig_P5) << 17);
    var2 = var2 + (((int64_t)bmp280_calib.dig_P4) << 35);
    var1 = ((var1 * var1 * bmp280_calib.dig_P3) >> 8) +
           ((var1 * bmp280_calib.dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * bmp280_calib.dig_P1 >> 33;

    if (var1 == 0)
        return 0;

    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (bmp280_calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = (bmp280_calib.dig_P8 * p) >> 19;

    p = ((p + var1 + var2) >> 8) + (((int64_t)bmp280_calib.dig_P7) << 4);
    return (uint32_t)p;
}


void BMP280_GetData(float *temperature,float *pressure,float *altitude)
{
    int32_t adc_T, adc_P;

    BMP280_ReadRaw(&adc_T, &adc_P);

    int32_t temp = BMP280_Compensate_T(adc_T);
    uint32_t press = BMP280_Compensate_P(adc_P);

    float P = press / 256.0f;   // Pa

    *temperature = temp / 100.0f;
    *pressure    = P;

    /* 气压高度计算 */
    //*altitude = 44330.0f * (1.0f - powf(P / SEA_LEVEL_PRESSURE, 0.1903f));
}


uint8_t BMP280_Init(void)
{
    uint8_t id;

    BMP280_ReadRegs(BMP280_REG_ID, &id, 1);
    if (id != 0x58)
        return 0;

    /* 复位 */
    BMP280_WriteReg(BMP280_REG_RESET, BMP280_RESET_VALUE);
    HAL_Delay(10);

    BMP280_ReadCalibration();

    /* config: t_sb=500ms, filter=x4 */
    BMP280_WriteReg(BMP280_REG_CONFIG,
                    (0x04 << 5) | (0x02 << 2));

    /* ctrl_meas: temp x2, press x16, normal mode */
    BMP280_WriteReg(BMP280_REG_CTRL_MEAS,
                    (0x02 << 5) | (0x05 << 2) | 0x03);

    return 1;
}