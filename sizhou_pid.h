#ifndef _SIZHOU_PID_H_
#define _SIZHOU_PID_H_

#include  "PID.h"

typedef struct PID_Data_struct{
	float x;
	float y;
	float z;
}PID_Data;

typedef struct sizhouPID_Data_struct{
	PIDparameter *x;
	PIDparameter *y;
	PIDparameter *z;
}sizhou_PID_parameter;


void sizhou_pidout(PID_Data *pos_rel, PID_Data *pos_val, PID_Data *pos_set, float *pid_int_limts, sizhou_PID_parameter *sizhou_pid_PosPar);
void sizhou_pidinn(PID_Data *ang_rel, PID_Data *ang_val, PID_Data *ang_set, float *pid_int_limts, sizhou_PID_parameter *sizhou_pid_AngPar);


#endif