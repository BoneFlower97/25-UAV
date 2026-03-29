#ifndef __STATE_CONTROL_H
#define __STATE_CONTROL_H
#include "stabilizer_types.h"
#include "global.h"

void stateControl(rc_ctrl_t rc_ctrl_con, sensorData_t *sensors_con, state_t *state_con,motor_out_t *motor_out,uint32_t tick_con);

#endif /*__STATE_CONTROL_H */

