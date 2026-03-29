#include "stabilizer.h"
#include "sensfusion6.h"
#include "gpio.h"
#include "tim.h"
#include "state_estimator.h"
#include "global.h"
#include "state_control.h"

static sensorData_t sensorData;
static control_t control;
static state_t 		state;		/*四轴姿态*/
static rc_ctrl_t rc_ctrl;
static control_command_t con;
static void send_pwm_motor(uint16_t ch1,uint16_t ch2,uint16_t ch3,uint16_t ch4)	
{
	pwm_set_ch1(ch1);
	pwm_set_ch2(ch2);
	pwm_set_ch3(ch3);
	pwm_set_ch4(ch4);
}

static void motor_init(uint16_t max,uint16_t min)
{
	set_sw_gpio(0);
	vTaskDelay(10);
	send_pwm_motor(max,max,max,max);
	set_sw_gpio(1);
	vTaskDelay(3000);
	send_pwm_motor(min,min,min,min);
	vTaskDelay(1000);
}
static void rc_control_update(void)
{
	if(state.isRCLocked != con.motor_lock && con.motor_lock == true)
	{
		state.isRCLocked = con.motor_lock;
		//set_sw_gpio(1);
		motor_init(200,100);
	}
	else if(state.isRCLocked != con.motor_lock && con.motor_lock == false)
	{
		state.isRCLocked = con.motor_lock;
		set_sw_gpio(0);		
	}
}

static void rc_ctrl_update(void)
{
//	if(state.isRCLocked != con.motor_lock && con.motor_lock == true)
//	{
//		state.isRCLocked = con.motor_lock;
//		set_sw_gpio(1);
//	}
//	else if(state.isRCLocked != con.motor_lock && con.motor_lock == false)
//	{
//		state.isRCLocked = con.motor_lock;
//		set_sw_gpio(0);		
//	}
	send_pwm_motor(rc_ctrl.ctrl_thr,rc_ctrl.ctrl_thr,rc_ctrl.ctrl_thr,rc_ctrl.ctrl_thr);
}

void stabilizerTask(void* param)
{
	uint32_t lastWakeTime = xTaskGetTickCount();
	uint32_t tick = 0;
	uint32_t err_tick = 0;
	motor_out_t motor_out;
	//set_sw_gpio(1);
	//motor_init(200,100);
	float last_asl;//代码结构有问题，这里暂存一下气压计高度数据，以免覆盖掉
	while(1)
	{
		vTaskDelayUntil(&lastWakeTime, MAIN_LOOP_DT);		/*1ms周期延时*/
		//获取6轴和气压数据（500Hz）
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			last_asl = sensorData.baro.asl;
			xQueueReceive(gyroDataQueue, &sensorData.gyro, 0);
			xQueueReceive(accelerometerDataQueue, &sensorData.acc, 0);
			xQueueReceive(barometerDataQueue, &sensorData.baro,0);
			sensorData.baro.asl = last_asl;
		}
				//四元数和欧拉角计算（250Hz）
		if (RATE_DO_EXECUTE(ATTITUDE_ESTIMAT_RATE, tick))
		{
			imuUpdate(sensorData.acc, sensorData.gyro, &state, ATTITUDE_ESTIMAT_DT);
		}
		
		if (RATE_DO_EXECUTE(RATE_10_HZ, tick))
		{
			baro_update(&sensorData);
		}
		if (RATE_DO_EXECUTE(RATE_100_HZ, tick))
		{
			nav_update(&sensorData,&state);
		}
		
		if(RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			send_pwm_motor(motor_out.ch1,motor_out.ch2,motor_out.ch3,motor_out.ch4);
			if(pdTRUE == xQueueReceive(controlcommandDataQueue, &rc_ctrl, 0))
			{
				
				//rc_ctrl_update();
				err_tick=0;
			}
			if(pdTRUE == xQueueReceive(rcctrlDataQueue, &con, 0))
			{
				err_tick=0;
				rc_control_update();
			}
			if(err_tick>5000)
			{
				err_tick = 0;
				set_sw_gpio(0);
				send_pwm_motor(100,100,100,100);
			}
		}

		
		stateControl(rc_ctrl, &sensorData, &state,&motor_out, tick);
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			if(state.isRCLocked == true && err_tick<500)
			{
				//send_pwm_motor(motor_out.ch1,motor_out.ch2,motor_out.ch3,motor_out.ch4);
			}
		}
		if (RATE_DO_EXECUTE(RATE_500_HZ, tick))
		{
			xQueueOverwrite(stateDataQueue, &state);
			xQueueOverwrite(sensorDataQueue, &sensorData);
			
		}
		err_tick++;
		tick++;
	}
}