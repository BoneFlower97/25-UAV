#include "sensfusion6.h"
//#include "global.h"

#include <math.h>
#include <stdbool.h>

#define DEG2RAD     0.01745329252f
#define RAD2DEG     57.29577951f

#define KP_DEFAULT   0.5f
#define KI_DEFAULT   0.02f

#define INT_LIM      0.2f

#define ACC_MIN_NORM 0.9f
#define ACC_MAX_NORM 1.1f

#define GRAVITY      9.80665f


static float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;
static float exInt = 0.0f, eyInt = 0.0f, ezInt = 0.0f;
static float rMat[3][3];
static bool matInited = false;

/* 快速倒平方根 */
static float invSqrt(float x)
{
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i >> 1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void computeRotationMatrix(void)
{
    float q0q0 = q0 * q0;
    float q1q1 = q1 * q1;
    float q2q2 = q2 * q2;
    float q3q3 = q3 * q3;

    rMat[0][0] = q0q0 + q1q1 - q2q2 - q3q3;
    rMat[0][1] = 2.0f * (q1*q2 - q0*q3);
    rMat[0][2] = 2.0f * (q1*q3 + q0*q2);

    rMat[1][0] = 2.0f * (q1*q2 + q0*q3);
    rMat[1][1] = q0q0 - q1q1 + q2q2 - q3q3;
    rMat[1][2] = 2.0f * (q2*q3 - q0*q1);

    rMat[2][0] = 2.0f * (q1*q3 - q0*q2);
    rMat[2][1] = 2.0f * (q2*q3 + q0*q1);
    rMat[2][2] = q0q0 - q1q1 - q2q2 + q3q3;
}

void imuUpdate(Axis3f acc, Axis3f gyro, state_t *state, float dt)
{
 float recipNorm;
    float ex, ey, ez;
    float halfT = 0.5f * dt;

    /* -------- 1. 加速度单位归一化 (m/s2 → g → unit vector) -------- */
    acc.x /= GRAVITY;
    acc.y /= GRAVITY;
    acc.z /= GRAVITY;

    float accNormSq = acc.x*acc.x + acc.y*acc.y + acc.z*acc.z;

    /* -------- 2. 加速度可信度判断 -------- */
    if (accNormSq > ACC_MIN_NORM*ACC_MIN_NORM &&
        accNormSq < ACC_MAX_NORM*ACC_MAX_NORM)
    {
        recipNorm = invSqrt(accNormSq);
        acc.x *= recipNorm;
        acc.y *= recipNorm;
        acc.z *= recipNorm;

        /* -------- 3. 由四元数计算“估计重力方向”（body frame） -------- */
        float vx = 2.0f * (q1*q3 - q0*q2);
        float vy = 2.0f * (q0*q1 + q2*q3);
        float vz = q0*q0 - q1*q1 - q2*q2 + q3*q3;

        /* -------- 4. 误差 = 测量重力 × 估计重力 -------- */
        ex = (acc.y * vz - acc.z * vy);
        ey = (acc.z * vx - acc.x * vz);
        ez = (acc.x * vy - acc.y * vx);

        /* -------- 5. 积分项（陀螺零偏补偿） -------- */
        exInt += KI_DEFAULT * ex * dt;
        eyInt += KI_DEFAULT * ey * dt;
        ezInt += KI_DEFAULT * ez * dt;

        /* 积分限幅 */
        if (exInt >  INT_LIM) exInt =  INT_LIM;
        if (exInt < -INT_LIM) exInt = -INT_LIM;
        if (eyInt >  INT_LIM) eyInt =  INT_LIM;
        if (eyInt < -INT_LIM) eyInt = -INT_LIM;
        if (ezInt >  INT_LIM) ezInt =  INT_LIM;
        if (ezInt < -INT_LIM) ezInt = -INT_LIM;

        /* -------- 6. PI 修正角速度 -------- */
        gyro.x += KP_DEFAULT * ex + exInt;
        gyro.y += KP_DEFAULT * ey + eyInt;
        gyro.z += KP_DEFAULT * ez + ezInt;
    }

    /* -------- 7. 四元数积分 -------- */
    float q0t = q0;
    float q1t = q1;
    float q2t = q2;
    float q3t = q3;

    q0 += (-q1t*gyro.x - q2t*gyro.y - q3t*gyro.z) * halfT;
    q1 += ( q0t*gyro.x + q2t*gyro.z - q3t*gyro.y) * halfT;
    q2 += ( q0t*gyro.y - q1t*gyro.z + q3t*gyro.x) * halfT;
    q3 += ( q0t*gyro.z + q1t*gyro.y - q2t*gyro.x) * halfT;

    /* -------- 8. 四元数归一化 -------- */
    recipNorm = invSqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    /* -------- 9. 方向余弦矩阵 -------- */
    computeRotationMatrix();

    /* -------- 10. 欧拉角输出 -------- */
    float sp = rMat[2][0];
    if (sp >  1.0f) sp =  1.0f;
    if (sp < -1.0f) sp = -1.0f;

    state->attitude.pitch = -asinf(sp) * RAD2DEG -1;//这里加减是为了补偿安装角度
    state->attitude.roll  = atan2f(rMat[2][1], rMat[2][2]) * RAD2DEG +1;
    state->attitude.yaw   = atan2f(rMat[1][0], rMat[0][0]) * RAD2DEG;
	}

	   /* ---------- 1. Body → World（直接使用 rMat） ---------- */
void acc_body_to_world(Axis3f *acc_world, Axis3f acc_body)
{
    /* 1. 首先转换到世界坐标系 */
    acc_world->x = rMat[0][0]*acc_body.x + rMat[0][1]*acc_body.y + rMat[0][2]*acc_body.z;
    acc_world->y = rMat[1][0]*acc_body.x + rMat[1][1]*acc_body.y + rMat[1][2]*acc_body.z;
    acc_world->z = rMat[2][0]*acc_body.x + rMat[2][1]*acc_body.y + rMat[2][2]*acc_body.z;
    const float g = 9.80665f;
    acc_world->x -= rMat[2][0] * g;
    acc_world->y -= rMat[2][1] * g;
    acc_world->z -= rMat[2][2] * g;
    
}