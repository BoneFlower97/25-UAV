#include "sensors.h"
#include "ICM20602.h"
#include "global.h"
#include "BMP280.h"

// ACC ±16g
// GYRO ±2000 dps
#define ACC_SENS   (8.0f * 9.80665f / 32768.0f)      // m/s^2 / LSB
#define GYRO_SENS  (1000.0f * 0.01745329252f / 32768.0f) // rad/s / LSB

#define GRAVITY       9.80665f
#define GYRO_STATIC_TH   0.10f        // rad/s
#define ACC_STATIC_MIN  (0.9f * GRAVITY)
#define ACC_STATIC_MAX  (1.1f * GRAVITY)
#define LPF_ALPHA        0.18f

// Yaw抑制相关参数
#define YAW_DEADZONE          0.002f    // rad/s (约0.1°/s)，忽略微小噪声
#define YAW_MOVING_SUPPRESS   0.8f      // 运动时抑制系数
#define YAW_STATIC_THRESHOLD  0.02f     // 单独检测Yaw静止的阈值

static float gyro_bias[3] = {0.0f, 0.0f, 0.0f};
static uint32_t gyro_bias_cnt = 0;
static sensorData_t sensors;

void processAccGyroMeasurements(uint8_t *buf)
{
    int16_t acc_raw[3];
    int16_t gyro_raw[3];
    float acc_tmp[3];
    float gyro_tmp[3];
    bool isStatic = false;
    static float z_scale_correction ;
    /* -------- 1. 字节拼接 -------- */
    acc_raw[0]  = (int16_t)((buf[0]  << 8) | buf[1]);
    acc_raw[1]  = (int16_t)((buf[2]  << 8) | buf[3]);
    acc_raw[2]  = (int16_t)((buf[4]  << 8) | buf[5]);
    
    gyro_raw[0] = (int16_t)((buf[8]  << 8) | buf[9]);
    gyro_raw[1] = (int16_t)((buf[10] << 8) | buf[11]);
    gyro_raw[2] = (int16_t)((buf[12] << 8) | buf[13]);
    
    /* -------- 2. 单位换算 -------- */
    acc_tmp[0]  = acc_raw[0]  * ACC_SENS;
    acc_tmp[1]  = acc_raw[1]  * ACC_SENS;
    acc_tmp[2]  = acc_raw[2]  * ACC_SENS;
    
    gyro_tmp[0] = gyro_raw[0] * GYRO_SENS;
    gyro_tmp[1] = gyro_raw[1] * GYRO_SENS;
    gyro_tmp[2] = gyro_raw[2] * GYRO_SENS;
    
    /* -------- 3. 静止检测 -------- */
    float acc_norm = sqrtf(acc_tmp[0]*acc_tmp[0] +
                           acc_tmp[1]*acc_tmp[1] +
                           acc_tmp[2]*acc_tmp[2]);
    
		// 当前输出：9.4669，期望：9.80665
    // 修正因子 = 期望 / 实际 = 9.80665 / 9.4669 = 1.0359
		z_scale_correction  = 9.80665/acc_norm;
		acc_tmp[2] *= z_scale_correction;
		
    float gyro_norm = fabsf(gyro_tmp[0]) +
                      fabsf(gyro_tmp[1]) +
                      fabsf(gyro_tmp[2]);
    
    // 单独检测Yaw轴是否静止
    float yaw_motion = fabsf(gyro_tmp[2]);
    bool yaw_is_static = (yaw_motion < YAW_STATIC_THRESHOLD);
    
    if ((gyro_norm < GYRO_STATIC_TH) &&
        (acc_norm > ACC_STATIC_MIN) &&
        (acc_norm < ACC_STATIC_MAX))
    {
        /* -------- 4. 在线零偏估计（递推平均） -------- */
        gyro_bias_cnt++;
        isStatic = true;
        
        // 限制最大计数，避免数值溢出
        if (gyro_bias_cnt > 10000) gyro_bias_cnt = 10000;
        
        for (int i = 0; i < 3; i++)
        {
            gyro_bias[i] += (gyro_tmp[i] - gyro_bias[i]) / gyro_bias_cnt;
        }
    }
    
    /* -------- 5. 零偏补偿 -------- */
    gyro_tmp[0] -= gyro_bias[0];
    gyro_tmp[1] -= gyro_bias[1];
    gyro_tmp[2] -= gyro_bias[2];
    
    /* -------- 6. Yaw抑制策略（关键修正）-------- */
    // 注意：这里的gyro_tmp[2]已经是零偏补偿后的值
    
    // 第一步：死区处理（忽略微小噪声）
    if (fabsf(gyro_tmp[2]) < YAW_DEADZONE)
    {
        gyro_tmp[2] = 0.0f;
    }
    else
    {
        // 第二步：根据状态动态抑制
        if (isStatic)
        {
            // 静止状态：完全归零
            gyro_tmp[2] = 0.0f;
            
            // 可选：在静止时让Yaw零偏快速收敛
            if (gyro_bias_cnt < 1000)  // 只在初始阶段快速收敛
            {
                // 轻微调整零偏，使其更快收敛到当前值
                gyro_bias[2] = gyro_bias[2] * 0.99f + gyro_tmp[2] * 0.01f;
            }
        }
        else if (yaw_is_static)
        {
            // Yaw轴单独静止但其他轴在动：中度抑制
            gyro_tmp[2] *= 0.3f;
        }
        else
        {
            // 运动状态：根据角速度大小动态抑制
            float abs_yaw = fabsf(gyro_tmp[2]);
            
            if (abs_yaw < 0.01f)        // < 0.57°/s，可能是噪声
            {
                gyro_tmp[2] *= 0.2f;    // 强抑制
            }
            else if (abs_yaw < 0.05f)   // < 2.86°/s，微小运动
            {
                gyro_tmp[2] *= 0.5f;    // 中度抑制
            }
            else if (abs_yaw < 0.1f)    // < 5.73°/s，可能是真实运动
            {
                gyro_tmp[2] *= 0.8f;    // 轻度抑制
            }
            else                        // > 5.73°/s，认为是真实旋转
            {
                gyro_tmp[2] *= 0.95f;   // 几乎不抑制
            }
        }
    }
    
    /* -------- 7. 一阶低通滤波 -------- */
    static uint8_t first = 1;
    
    if (first)
    {
        sensors.acc.x  = acc_tmp[0];
        sensors.acc.y  = acc_tmp[1];
        sensors.acc.z  = acc_tmp[2];
        
        sensors.gyro.x = gyro_tmp[0];
        sensors.gyro.y = gyro_tmp[1];
        sensors.gyro.z = gyro_tmp[2];
        
        first = 0;
    }
    else
    {
        sensors.acc.x  += LPF_ALPHA * (acc_tmp[0]  - sensors.acc.x);
        sensors.acc.y  += LPF_ALPHA * (acc_tmp[1]  - sensors.acc.y);
        sensors.acc.z  += LPF_ALPHA * (acc_tmp[2]  - sensors.acc.z);
        
        sensors.gyro.x += LPF_ALPHA * (gyro_tmp[0] - sensors.gyro.x);
        sensors.gyro.y += LPF_ALPHA * (gyro_tmp[1] - sensors.gyro.y);
        sensors.gyro.z += LPF_ALPHA * (gyro_tmp[2] - sensors.gyro.z);
    }
    
   
}

/* 传感器任务 */
void sensorsTask(void *param)
{
    ICM20602_Init();  
		BMP280_Init();
    uint8_t imu_buf[14];                                                                                             
    
    while (1)
    {
        if(pdTRUE == xSemaphoreTake(icmDataReady, portMAX_DELAY))
        {
            vTaskSuspendAll();  /*确保同一时刻把数据放入队列中*/
            ICM20602_ReadRegs(0x3B, imu_buf, 14);
            processAccGyroMeasurements(&imu_buf[0]);
					
            BMP280_GetData(&sensors.baro.temperature,&sensors.baro.pressure,&sensors.baro.asl);
            xQueueOverwrite(accelerometerDataQueue, &sensors.acc);
            xQueueOverwrite(gyroDataQueue, &sensors.gyro);
            xQueueOverwrite(barometerDataQueue, &sensors.baro);
            xTaskResumeAll();
        }
    }
}