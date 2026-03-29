#ifndef __STATE_ESTIMATOR_H
#define __STATE_ESTIMATOR_H
#include "stabilizer_types.h"
#include <stdbool.h>

void nav_update(sensorData_t* sensorData, state_t* state);
void baro_update(sensorData_t *sensorData);
#endif /* __STATE_ESTIMATOR_H */


