#ifndef  __PID_H__
#define  __PID_H__




typedef struct PIDparameter_struct{
    float kp, ki, kd;
    float err, err1, err2;
    float errint;
}PIDparameter;



float PID(float set, float actual, PIDparameter *haha);

#endif
