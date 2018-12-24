#include "PID.h"

//PID算法
float PID(float set, float actual, PIDparameter *haha){
    haha->err2 = haha->err1;
    haha->err1 = haha->err;
    
    haha->err = set - actual;
    haha->errint += haha->err;
	
    return haha->kp*haha->err + haha->ki*haha->errint + haha->kd*(3 * haha->err - 4 * haha->err1 + haha->err2);
}

/*

//三个方向都算一遍内外环PID  (ง •_•)ง
void pid_motor(float *pid_out, float *set, float *actual, PIDparameter *(pid_value[])){
    unsigned char i=0;
    float w_test[3] = {0,0,0};
    
    for (i=0; i<3; i++){
        pid_out[i] = PID(PID(set[i], actual[i], pid_value[i]),
                         w_test[i], pid_value[i+3]);
    }
}


*/


