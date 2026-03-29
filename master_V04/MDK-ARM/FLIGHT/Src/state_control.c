#include <math.h>
#include "state_control.h"

typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float last_error;
    float last_measure;  // 保存上一次测量值，用于微分计算

    float out_min;
    float out_max;
    float integral_limit; // 积分限幅
    float deadband;       // 新增：误差死区阈值（根据场景设置，比如角度环0.1°，角速度环0.5°/s）
} pid_t;

static float test1;
static float test2;
static float test3;
static float test4;
	
// 初始化时给不同PID设置对应死区（示例值，可根据实际调试调整）
static pid_t pid_roll_angle  = {.kp = 0.08f, .ki = 0.005f, .kd = 0.0f, .out_min = -50.0f, .out_max = 50.0f, .integral_limit = 50.0f, .deadband = 0.01f};  // 角度环死区0.1°，输出限幅调整为电机控制量范围
static pid_t pid_pitch_angle = {.kp = 0.10f, .ki = 0.005f, .kd = 0.0f, .out_min = -50.0f, .out_max = 50.0f, .integral_limit = 50.0f, .deadband = 0.01f};
static pid_t pid_yaw_angle   = {.kp = 0.00f, .ki = 0.000f, .kd = 0.0f, .out_min = -30.0f, .out_max = 30.0f, .integral_limit = 40.0f, .deadband = 0.01f};
// 角速度环PID保留定义，方便后续恢复，仅注释使用逻辑
static pid_t pid_roll_rate  = {.kp = 6.5f, .ki = 0.005f, .kd = 0.06f, .out_min = -50.0f, .out_max = 50.0f, .integral_limit = 50.0f, .deadband = 0.01f};
static pid_t pid_pitch_rate = {.kp = 6.5f, .ki = 0.005f, .kd = 0.06f, .out_min = -50.0f, .out_max = 50.0f, .integral_limit = 50.0f, .deadband = 0.01f};
static pid_t pid_yaw_rate   = {.kp = 5.0f, .ki = 0.005f, .kd = 0.02f, .out_min = -50.0f, .out_max = 50.0f, .integral_limit = 50.0f, .deadband = 0.1f};

// 优化后的PID计算函数（增加误差死区）
static float pid_calculate(pid_t *pid, float target, float measure, float dt)
{
    // 防止除零错误
    if (dt <= 0.0f || dt > 0.1f) { 
        pid->last_error = target - measure;
        pid->last_measure = measure;
        return 0.0f;
    }

    float error = target - measure;

    // ===== 新增：误差死区处理 =====
    // 当误差绝对值小于死区阈值时，认为误差为0
    if (fabsf(error) < pid->deadband) {
        error = 0.0f;
        // 死区内积分项停止累积（防止积分漂移）
        pid->integral *= 0.95f; // 可选：积分项小幅衰减，加快清零
    }

    // 1. 比例项
    float p_term = pid->kp * error;

    // 2. 积分项（带抗饱和，且仅在死区外累积）
    if (fabsf(error) >= pid->deadband) { // 仅死区外累积积分
        pid->integral += error * dt;
    }
    // 积分限幅，防止积分饱和
    if (pid->integral > pid->integral_limit) {
        pid->integral = pid->integral_limit;
    } else if (pid->integral < -pid->integral_limit) {
        pid->integral = -pid->integral_limit;
    }
    float i_term = pid->ki * pid->integral;

    // 3. 微分项（基于测量值计算，避免微分突变；死区内微分项为0）
    float d_term = 0.0f;
    if (fabsf(error) >= pid->deadband) { // 仅死区外计算微分
        d_term = pid->kd * (measure - pid->last_measure) / dt;
    }
    // 微分负号：因为d(误差)/dt = d(目标-测量)/dt = -d(测量)/dt

    // 总输出
    float output = p_term + i_term - d_term;

    // 输出限幅
    if (output > pid->out_max) {
        output = pid->out_max;
        // 积分反饱和（输出限幅时停止积分）
        if (error > 0) pid->integral -= error * dt;
    } else if (output < pid->out_min) {
        output = pid->out_min;
        // 积分反饱和
        if (error < 0) pid->integral -= error * dt;
    }

    // 更新历史值
    pid->last_error = error;
    pid->last_measure = measure;

    return output;
}
void stateControl(rc_ctrl_t rc_ctrl_con,
                  sensorData_t *sensors_con,
                  state_t *state_con,
                  motor_out_t *motor_out_con,
                  uint32_t tick_con)
{
	float roll_rate_target;
	float pitch_rate_target;
	float yaw_rate_target;
	
    // 5Hz更新PID参数（角度环+角速度环，角速度环参数保留，方便后续恢复）
    if (RATE_DO_EXECUTE(RATE_5_HZ, tick_con))
    {
        // 角度环 PID 参数赋值（对应 PID_4/5/6 组，缩放放大10倍）
//        pid_roll_angle.kp = (float)pid_group.PID_4_P / 1000.0f;   // 原/1000 → 现/100，放大10倍
//        pid_roll_angle.ki = (float)pid_group.PID_4_I / 1000.0f;
//        pid_roll_angle.kd = (float)pid_group.PID_4_D / 1000.0f;  // KD仍÷1000，避免噪声放大

//        pid_pitch_angle.kp = (float)pid_group.PID_5_P / 1000.0f;
//        pid_pitch_angle.ki = (float)pid_group.PID_5_I / 1000.0f;
//        pid_pitch_angle.kd = (float)pid_group.PID_5_D / 1000.0f;

//        pid_yaw_angle.kp = (float)pid_group.PID_6_P / 1000.0f;
//        pid_yaw_angle.ki = (float)pid_group.PID_6_I / 1000.0f;
//        pid_yaw_angle.kd = (float)pid_group.PID_6_D / 1000.0f;

//        // 角速度环 PID 参数赋值保留，注释仅作标记，方便后续恢复
//        
//        pid_roll_rate.kp = (float)pid_group.PID_1_P / 1000.0f;
//        pid_roll_rate.ki = (float)pid_group.PID_1_I / 1000.0f;
//        pid_roll_rate.kd = (float)pid_group.PID_1_D / 1000.0f;

//        pid_pitch_rate.kp = (float)pid_group.PID_2_P / 1000.0f;
//        pid_pitch_rate.ki = (float)pid_group.PID_2_I / 1000.0f;
//        pid_pitch_rate.kd = (float)pid_group.PID_2_D / 1000.0f;

//        pid_yaw_rate.kp = (float)pid_group.PID_3_P / 1000.0f;
//        pid_yaw_rate.ki = (float)pid_group.PID_3_I / 1000.0f;
//        pid_yaw_rate.kd = (float)pid_group.PID_3_D / 1000.0f;
        
    }
    // 250Hz执行PID控制逻辑（仅保留角度环）
    if (RATE_DO_EXECUTE(RATE_250_HZ, tick_con))
    {
        const float dt = 1.0f / 250.0f;

        /* ========= 1. 目标角度：遥控器指令转换为角度目标 ========= */
        // 横滚目标角度（摇杆范围映射到角度范围，比如±20°，可根据实际调试调整）
        float roll_target  = (float)rc_ctrl_con.ctrl_rol / 100.0f;   // ÷25 → 摇杆500对应20°
        // 俯仰目标角度
        float pitch_target = (float)rc_ctrl_con.ctrl_pit / 100.0f; 
			
        float yaw_target;

        /* ========= 2. 当前角度（来自姿态解算） ========= */
        float roll_meas  = state_con->attitude.roll;   // 横滚角度（°）
        float pitch_meas = state_con->attitude.pitch;  // 俯仰角度（°）
        float yaw_meas   = state_con->attitude.yaw;    // 偏航角度（°）
				
        /* ========= 3. 角度环PID → 直接输出电机控制量 ========= */
        // 角度环输出直接作为电机控制量，限幅调整为±50（匹配原角速度环输出范围）
        //float roll_out  = pid_calculate(&pid_roll_angle, roll_target, roll_meas, dt);
        //float pitch_out = pid_calculate(&pid_pitch_angle, pitch_target, pitch_meas, dt);
        //float yaw_out   = pid_calculate(&pid_yaw_angle, yaw_target, yaw_meas, dt);

        /* ========= 注释：角速度环相关逻辑（保留大纲，方便恢复） ========= */
        
        // 角度环输出作为角速度目标（原串级逻辑，注释保留）
				//if(pid_group.PID_4_P !=0)
				if(1)	
				{
					 roll_rate_target  = pid_calculate(&pid_roll_angle, roll_target, roll_meas, dt);
					 pitch_rate_target = pid_calculate(&pid_pitch_angle, pitch_target, pitch_meas, dt);
					 //yaw_rate_target   = pid_calculate(&pid_yaw_angle, yaw_target, yaw_meas, dt);
						
				}
				else
				{
					 roll_rate_target  = roll_target;
					 pitch_rate_target = pitch_target;
					 yaw_rate_target   = yaw_target;
				}
				yaw_rate_target   = (float)rc_ctrl_con.ctrl_yawdps/100.0f;
        // 当前角速度（来自陀螺仪）
        float roll_rate_meas  = sensors_con->gyro.x;   // 横滚角速度（°/s）
        float pitch_rate_meas = sensors_con->gyro.y;   // 俯仰角速度（°/s）
        float yaw_rate_meas   = sensors_con->gyro.z;   // 偏航角速度（°/s）

        // 内环：角速度PID → 输出电机控制量
        float roll_out = pid_calculate(&pid_roll_rate, roll_rate_target, roll_rate_meas, dt);
        float pitch_out = pid_calculate(&pid_pitch_rate, pitch_rate_target, pitch_rate_meas, dt);
        float yaw_out = pid_calculate(&pid_yaw_rate, yaw_rate_target, yaw_rate_meas, dt);
        

        /* ========= 6. 油门处理 ========= */
        float throttle = rc_ctrl_con.ctrl_thr;

        /* ========= 7. X型混控 ========= */
        float motor[4];
        motor[0] = throttle + pitch_out + roll_out + yaw_out;  // 电机1
        motor[1] = throttle + pitch_out - roll_out - yaw_out;  // 电机2
        motor[2] = throttle - pitch_out - roll_out + yaw_out;  // 电机3
        motor[3] = throttle - pitch_out + roll_out - yaw_out;  // 电机4

        /* ========= 8. 电机输出限幅 ========= */
        // 最小油门保护（低于110时电机输出最小值）
        if (throttle < 110)
        {
            for (int i = 0; i < 4; i++)
                motor[i] = 100;
        }
        else
        {
            // 电机输出限幅（100~200，匹配电调输入范围）
            for (int i = 0; i < 4; i++)
            {
                if (motor[i] > 200) motor[i] = 200;
                if (motor[i] < 100) motor[i] = 100;
            }
        }

        /* ========= 9. 赋值电机输出 ========= */
        motor_out_con->ch1 = motor[2];
        motor_out_con->ch2 = motor[3];
        motor_out_con->ch3 = motor[0];
        motor_out_con->ch4 = motor[1];

        // 测试用变量：记录目标/实际角度、角速度
				test1 =  yaw_target;
				test2 =  yaw_target;
				test3 =  yaw_out;
				test4 =  throttle;
    }
}