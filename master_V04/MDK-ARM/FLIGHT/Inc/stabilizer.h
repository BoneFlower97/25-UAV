#ifndef __STABALIZER_H
#define __STABALIZER_H

#define ATTITUDE_ESTIMAT_RATE	RATE_250_HZ	//姿态解算速率
#define ATTITUDE_ESTIMAT_DT		(1.0/RATE_250_HZ)


void stabilizerTask(void* param);

#endif /* __STABALIZER_H */
