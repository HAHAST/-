#ifndef  __PID_H__
#define  __PID_H__

typedef struct{
	float kp, ki, kd;
	float err, err1, err2;
	float errint;
}PIDparameter;

#endif
