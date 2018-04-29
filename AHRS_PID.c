#include <math.h>
#include "AHRS_PID.h"

//姿态解算部分-------------------------------------------------------------------------------------------


float s = 1, x = 0, y = 0, z = 0;  //四元数 
float pitch = 0, yaw = 0, roll = 0; 
float w_x0 = 0, w_y0 = 0, w_z0 = 0; 
float w[3] = {0, 0, 0};
float dt = 10;


//欧拉角转四元数
void EtoQ(){
    float norm;

    s = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - sin(yaw / 2)*sin(roll / 2)*sin(pitch / 2);
    x = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) - sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
    y = cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2);
    z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*cos(yaw / 2);

    norm = sqrt(s*s + x * x + y * y + z * z);
    s = s / norm;
    x = x / norm;
    y = y / norm;
    z = z / norm;
}

//四元数转欧拉角
void QtoE(){
    pitch = asin(2 * (z*y + s * x));
    yaw = atan2(2 * (z*s - y * x), (1 - 2 * x*x - 2 * z*z));
    roll = atan2(2 * (s*y - x * z), (1 - 2 * y*y - 2 * x*x));
}


//姿态更新
void updata(void){
    float cup0, cup1, cup2, cup3, norm;
	
    cup0 = s - 0.5*(w[0]*x + w[1]*y + w[2]*z)*dt;
    cup1 = x + 0.5*(w[0]*s + w[2]*y - w[1]*z)*dt;
    cup2 = y + 0.5*(w[1]*s - w[2]*x + w[0]*z)*dt;
    cup3 = z + 0.5*(w[2]*s + w[1]*x - w[0]*y)*dt;
    norm = sqrt(cup0*cup0 + cup1*cup1 + cup2*cup2 + cup3*cup3);
    
    s = cup0 / norm;
    x = cup1 / norm;
    y = cup2 / norm;
    z = cup3 / norm;
}


//互补滤波（真叫这个名字吗）
#define Ki   0.001f
#define Kp   0.8f
float eInt_x = 0, eInt_y = 0, eInt_z = 0; 
void COM_FILT(Mpu9255_Data *accel, Mpu9255_Data *gyro)
{
    float e_x, e_y, e_z;
    float g_x, g_y, g_z;
    float a_x, a_y, a_z;
    float norm;

    norm = sqrt(accel->x*accel->x + accel->y*accel->y + accel->z*accel->z);
    a_x = accel->x / norm;
    a_y = accel->y / norm;
    a_z = accel->z / norm; 

    g_x = 2 * (x*z - s*y);
    g_y = 2 * (y*z + s*x);
    g_z = 1 - 2 * (x*x + y*y);   

    e_x = a_y*g_z - a_z*g_y;
    e_y = a_z*g_x - a_x*g_z;
    e_z = a_x*g_y - a_y*g_x;

    eInt_x = eInt_x + e_x*Ki;
    eInt_y = eInt_y + e_y*Ki;
    eInt_z = eInt_z + e_z*Ki;

    w[0] = gyro->x -w_x0 + Kp*e_x + eInt_x;
    w[1] = gyro->y -w_y0 + Kp*e_y + eInt_y;
    w[2] = gyro->z -w_z0 + Kp*e_z + eInt_z;
}
/*
//卡尔曼滤波 _(:з」∠)_
void KF(){
	
}
*/



//PID部分--------------------------------------------------

//PID算法
float PID(float set, float actual, PIDparameter *haha){
    haha->err2 = haha->err1;
    haha->err1 = haha->err;
    
    haha->err = set - actual;
    haha->errint += haha->err;
	
    return haha->kp*haha->err + haha->ki*haha->errint + haha->kd*(3 * haha->err - 4 * haha->err1 + haha->err2);
}

//三个方向都算一遍内外环PID  (ง •_•)ง
void pid_motor(float *pid_out, float *set, float *actual, PIDparameter *(pid_value[])){
    unsigned char i=0;
    float w_test[3] = {0,0,0};
    
    for (i=0; i<3; i++){
        pid_out[i] = PID(PID(set[i], actual[i], pid_value[i]),
                         w_test[i], pid_value[i+3]);
    }
}

//PID的输出与PWM对应关系----------------------------------------------------------------------------
float *motor(float *Motor, float *pid_out, float thu){
//PID的输出           yaw           pitch        roll    
    Motor[1] = thu + pid_out[2] + pid_out[0] + pid_out[1];
    Motor[2] = thu - pid_out[2] + pid_out[0] - pid_out[1];
    Motor[3] = thu + pid_out[2] - pid_out[0] - pid_out[1];
    Motor[4] = thu - pid_out[2] - pid_out[0] + pid_out[1];
    
    return Motor;
}
//---------------------------------------------------------------------------------


//唯二的对外可见函数
float *AHRS(float *pwm, Mpu9255_Data *accel, Mpu9255_Data *gyro, float nrf[], PIDparameter *pid_QAQ){
    float th[3];
    float th_set[3];
    float pid[3];
    float prowe = nrf[0];
    
    th_set[0] = nrf[1];
    th_set[1] = nrf[2];
    th_set[2] = 0;
    
    COM_FILT(accel, gyro);
    updata();
    QtoE();
    
    th[0] = pitch;
    th[1] = roll;
    th[2] = yaw;
    pid_motor(pid, th_set, th, pid_QAQ);
    
    return motor(pwm, pid, prowe);
}


//唯二对外可见函数
void init_AHRS(Mpu9255_Data *accel, Mpu9255_Data *gyro, float time){
    pitch = atan2(accel->y, accel->z);
    roll = atan2(accel->x, accel->z);
    E_Q();
    w_x0 = gyro->x;
    w_y0 = gyro->y;
    w_z0 = gyro->z;
    
    dt = time;
}

