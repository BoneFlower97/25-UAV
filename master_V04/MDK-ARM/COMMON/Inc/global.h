#ifndef __GLOBAL_H
#define __GLOBAL_H
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "sensors_types.h"
#include "stabilizer_types.h"
#include "filter.h"

typedef struct
{
    int16_t ctrl_rol;     // ADC 最小值
    int16_t ctrl_pit;     // ADC 最大值
    int16_t ctrl_thr;    // 输出最小值
    int16_t ctrl_yawdps;    // 输出最大值
}rc_ctrl_t;
typedef struct
{
	bool motor_lock;
}
control_command_t;

typedef struct
{
	uint16_t ch1;
	uint16_t ch2;
	uint16_t ch3;
	uint16_t ch4;
}motor_out_t;

typedef struct {
    union {
        struct {
            // 角速度环 ROL (横滚)
            uint16_t PID_1_P; // 第1组PID：P：角速度环：ROL
            uint16_t PID_1_I; // 第1组PID：I：角速度环：ROL
            uint16_t PID_1_D; // 第1组PID：D：角速度环：ROL

            // 角速度环 PIT (俯仰)
            uint16_t PID_2_P; // 第2组PID：P：角速度环：PIT
            uint16_t PID_2_I; // 第2组PID：I：角速度环：PIT
            uint16_t PID_2_D; // 第2组PID：D：角速度环：PIT

            // 角速度环 YAW (偏航)
            uint16_t PID_3_P; // 第3组PID：P：角速度环：YAW
            uint16_t PID_3_I; // 第3组PID：I：角速度环：YAW
            uint16_t PID_3_D; // 第3组PID：D：角速度环：YAW

            // 角度环 ROL (横滚)
            uint16_t PID_4_P; // 第4组PID：P：角度环：ROL
            uint16_t PID_4_I; // 第4组PID：I：角度环：ROL
            uint16_t PID_4_D; // 第4组PID：D：角度环：ROL

            // 角度环 PIT (俯仰)
            uint16_t PID_5_P; // 第5组PID：P：角度环：PIT
            uint16_t PID_5_I; // 第5组PID：I：角度环：PIT
            uint16_t PID_5_D; // 第5组PID：D：角度环：PIT

            // 角度环 YAW (偏航)
            uint16_t PID_6_P; // 第6组PID：P：角度环：YAW
            uint16_t PID_6_I; // 第6组PID：I：角度环：YAW
            uint16_t PID_6_D; // 第6组PID：D：角度环：YAW
        };
        // 数组形式：与上面的参数一一对应，共享同一块内存
        uint16_t pid_array[18]; // 18个参数：6组×3个(P/I/D)
    };
} pid_group_t;
extern pid_group_t pid_group;
extern  SemaphoreHandle_t sensorsDataReady;
extern SemaphoreHandle_t icmDataReady;
//extern SemaphoreHandle_t uart1Mutex;
extern SemaphoreHandle_t uart1TxDone;

extern xQueueHandle accelerometerDataQueue;
extern xQueueHandle barometerDataQueue;
extern xQueueHandle gyroDataQueue;
extern xQueueHandle stateDataQueue;
extern xQueueHandle sensorDataQueue;
extern xQueueHandle uartRxQueue;
extern xQueueHandle rcctrlDataQueue;
extern xQueueHandle controlcommandDataQueue;
extern bool link_sta;

void dio_EXTI(void);
void SX126xTimerIrqHandler();

#endif
