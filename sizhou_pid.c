#include  "PID.h"
#include  "sizhou_pid.h"

float pid_int_windup(float set, float actual, PIDparameter *haha, float pid_int_limt){
    if (haha->errint > pid_int_limt){
        haha->errint = pid_int_limt;
    }
    
    if (haha->errint < 0){
        haha->errint = 0;
    }
    
    return PID(set, actual, haha);
}

void sizhou_pidout(PID_Data *pos_rel, PID_Data *pos_val, PID_Data *pos_set, float pid_int_limts[], sizhou_PID_parameter *sizhou_pid_PosPar){ 
    
    pos_rel->x = pid_int_windup(pos_set->x, pos_val->x, sizhou_pid_PosPar->x, pid_int_limts[0]);
    pos_rel->y = pid_int_windup(pos_set->y, pos_val->y, sizhou_pid_PosPar->y, pid_int_limts[1]);
    pos_rel->z = pid_int_windup(pos_set->z, pos_val->z, sizhou_pid_PosPar->z, pid_int_limts[2]);
}



void sizhou_pidinn(PID_Data *ang_rel, PID_Data *ang_val, PID_Data *ang_set, float pid_int_limts[], sizhou_PID_parameter *sizhou_pid_AngPar){   
    
    ang_rel->x = pid_int_windup(ang_set->x, ang_val->x, sizhou_pid_AngPar->x, pid_int_limts[0]);
    ang_rel->y = pid_int_windup(ang_set->y, ang_val->y, sizhou_pid_AngPar->y, pid_int_limts[1]);
    ang_rel->z = pid_int_windup(ang_set->z, ang_val->z, sizhou_pid_AngPar->z, pid_int_limts[2]);
}











