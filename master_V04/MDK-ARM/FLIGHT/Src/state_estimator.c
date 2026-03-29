#include "state_estimator.h"
#include "sensfusion6.h"
#include <math.h>
#include <string.h>

#define NAV_DT              0.01f      // 100 Hz, 10ms
#define GRAVITY             9.80665f   // m/s2

#define ACC_TRUST_THRESH    0.3f       // m/s2
#define VEL_DAMP            0.995f     // 速度阻尼系数
#define ZUPT_THRESH         0.2f       // 零速检测阈值

//#define BARO_LPF_K          0.05f      // 气压计低通滤波系数
#define BARO_FUSE_K         0.02f      // 气压融合权重


#define BARO_MEDIAN_SIZE   100
#define BARO_LPF_K         0.02f   // 10 Hz 下 ≈ 5 秒时间常数
#define BARO_INIT_COUNT    20      // 2 秒初始化

// 导航状态结构
typedef struct {
    Axis3f vel;        // 速度 (m/s)
    Axis3f pos;        // 位置 (m)
    float baro_base;   // 气压基准 (m)
    float baro_filt;   // 滤波后的气压高度 (m)
    uint8_t initialized;
} nav_state_t;

static nav_state_t nav = {0};

void nav_update(sensorData_t* sensorData, state_t* state)
{
    static int initialized = 0;
    static Axis3f vel = {0};
    static Axis3f pos = {0};
    static float baroBase = 0.0f;
    static float baroFilt = 0.0f;
    
    // 重力补偿变量
    static float gravity_bias_x = 0.0f;
    static float gravity_bias_y = 0.0f;
    static float gravity_bias_z = 0.0f;

    /* ========== 1. 初始化 ========== */
    if (!initialized)
    {
        static int cnt = 0;
        baroBase += sensorData->baro.asl;
        cnt++;

        if (cnt >= 100)
        {
            baroBase /= cnt;
            memset(&vel, 0, sizeof(vel));
            memset(&pos, 0, sizeof(pos));
            gravity_bias_x = gravity_bias_y = gravity_bias_z = 0.0f;
            initialized = 1;
        }
        return;
    }

    /* ========== 2. 转换到世界坐标系 ========== */
    Axis3f accWorld;
    acc_body_to_world(&accWorld, sensorData->acc);

    /* ========== 3. 重力补偿 ========== */
    // 静态学习重力偏差
    float accNorm = sqrtf(accWorld.x*accWorld.x + 
                         accWorld.y*accWorld.y + 
                         accWorld.z*accWorld.z);
    float gyroNorm = fabsf(sensorData->gyro.x) +
                     fabsf(sensorData->gyro.y) +
                     fabsf(sensorData->gyro.z);
    
    bool isStatic = (accNorm < 0.3f) && (gyroNorm < 0.05f);
    
    if (isStatic)
    {
        // 学习重力偏差（低通滤波）
        float alpha = 0.01f;  // 学习率
        gravity_bias_x = gravity_bias_x * (1.0f - alpha) + accWorld.x * alpha;
        gravity_bias_y = gravity_bias_y * (1.0f - alpha) + accWorld.y * alpha;
        gravity_bias_z = gravity_bias_z * (1.0f - alpha) + accWorld.z * alpha;
        
        // 补偿
        accWorld.x -= gravity_bias_x;
        accWorld.y -= gravity_bias_y;
        accWorld.z -= gravity_bias_z;
            // 关键：Z轴额外补偿，强制去除剩余重力
				float g_remain = accWorld.z;
				if (fabsf(g_remain) > 0.05f) {
						accWorld.z -= g_remain * 0.1f;  // 缓慢修正
				}
        // 零速时速度强制归零
        vel.x = vel.y = vel.z = 0.0f;
    }

    /* ========== 4. 速度积分 ========== */
    if (!isStatic && !state->isRCLocked)
    {
        vel.x += accWorld.x * NAV_DT;
        vel.y += accWorld.y * NAV_DT;
        vel.z += accWorld.z * NAV_DT;
    }

    // 速度阻尼（防止微小误差累积）
    vel.x *= VEL_DAMP;
    vel.y *= VEL_DAMP;
    vel.z *= VEL_DAMP;

    /* ========== 5. 位置积分 ========== */
    pos.x += vel.x * NAV_DT;
    pos.y += vel.y * NAV_DT;
    pos.z += vel.z * NAV_DT;

    /* ========== 6. 气压融合 ========== */
    float baroRel = sensorData->baro.asl - baroBase;
    baroFilt = baroFilt * (1.0f - BARO_LPF_K) + baroRel * BARO_LPF_K;
    
    // 气压融合到Z轴（权重很小，仅用于长期稳定）
    pos.z = pos.z * (1.0f - BARO_FUSE_K) + baroFilt * BARO_FUSE_K;

    /* ========== 7. 外部锁定处理 ========== */
    if (state->isRCLocked)
    {
        vel.x = vel.y = vel.z = 0.0f;
        pos.x = pos.y = 0.0f;
        pos.z = 0.0f;
        baroBase = sensorData->baro.asl;
        baroFilt = 0.0f;
        gravity_bias_x = gravity_bias_y = gravity_bias_z = 0.0f;
    }

    /* ========== 8. 输出（单位：米） ========== */
    state->acc.x = accWorld.x;
    state->acc.y = accWorld.y;
    state->acc.z = accWorld.z;
    
    state->velocity.x = vel.x;
    state->velocity.y = vel.y;
    state->velocity.z = vel.z;
    
    state->position.x = pos.x;
    state->position.y = pos.y;
    state->position.z = pos.z;
    

}

static float median5(float *v)
{
    float a[5];
    memcpy(a, v, sizeof(a));

    for (int i = 0; i < 4; i++)
        for (int j = i + 1; j < 5; j++)
            if (a[i] > a[j]) {
                float t = a[i];
                a[i] = a[j];
                a[j] = t;
            }

    return a[2];
}

static float pressure_to_height(float pressure_pa)
{
    // 国际标准大气模型
    return 44330.0f * (1.0f - powf(pressure_pa / 101325.0f, 0.19029495f));
}

void baro_update(sensorData_t *sensorData)
{
    static uint8_t initialized = 0;

    static float pressure_buf[BARO_MEDIAN_SIZE] = {0};
    static uint8_t buf_idx = 0;

    static float baro_base = 0.0f;
    static float baro_lpf = 0.0f;

    /* ========== 1. 缓冲原始气压（Pa） ========== */
    pressure_buf[buf_idx++] = sensorData->baro.pressure;
    if (buf_idx >= BARO_MEDIAN_SIZE)
        buf_idx = 0;

    /* 缓冲未满前不输出 */
    static uint8_t sample_cnt = 0;
    if (sample_cnt < BARO_MEDIAN_SIZE)
    {
        sample_cnt++;
        return;
    }

    /* ========== 2. 中值滤波（抗桨风） ========== */
    float pressure_med = median5(pressure_buf);

    /* ========== 3. 气压 → 高度（米） ========== */
    float height_raw = pressure_to_height(pressure_med);

    /* ========== 4. 初始化低通状态 ========== */
    if (!initialized)
    {
        baro_lpf = height_raw;
        initialized = 1;
    }

    /* ========== 5. 慢低通（只保留趋势） ========== */
    baro_lpf += BARO_LPF_K * (height_raw - baro_lpf);

    /* ========== 6. 输出：绝对气压高度 ========== */
    sensorData->baro.asl = baro_lpf;
}
/* ========== 辅助函数：重置导航 ========== */
void nav_reset(void)
{
    nav.initialized = 0;
    memset(&nav, 0, sizeof(nav_state_t));
}

/* ========== 辅助函数：设置当前位置 ========== */
void nav_set_position(float x, float y, float z)
{
    nav.pos.x = x;
    nav.pos.y = y;
    nav.pos.z = z;
}

/* ========== 辅助函数：设置当前速度 ========== */
void nav_set_velocity(float vx, float vy, float vz)
{
    nav.vel.x = vx;
    nav.vel.y = vy;
    nav.vel.z = vz;
}