/*FreeRTOS���ͷ�ļ�*/
#include <stdbool.h>
#include <math.h>

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
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
	bool up;
	bool down;
	bool right;
	bool re;
}
rc_control_command_t;


extern xQueueHandle rcctrlDataQueue;
extern xQueueHandle controlcommandDataQueue;
extern xQueueHandle rccontrolDataQueue;

void SX126xTimerIrqHandler();
void dio_EXTI(void);

