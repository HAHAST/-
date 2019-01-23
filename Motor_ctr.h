#ifndef  __MOTOR_CTR_H__
#define  __MOTOR_CTR_H__


void ctr_motor(float *pid_out, float thu, void (*motor_power)(unsigned int, unsigned int, unsigned int, unsigned int) reentrant);

#endif