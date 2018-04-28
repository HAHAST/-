#include <math.h>
#include "AHRS_PID.h"

//四轴姿态参数

typedef struct mpu9255_data_struct{
    float x;
    float y;
    float z;
}mpu9255_data;


float s = 1, x = 0, y = 0, z = 0;  //四元数 
float pitch = 0, yaw = 0, roll = 0; 
float w_x0 = 0, w_y0 = 0, w_z0 = 0; 
float w[3] = {0, 0, 0};         

void EtoQ()  //欧拉角转四元数
{
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
void updata(float dt){
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

//互补滤波（真的吗）
void COM_FILT(mpu9255_data accel, mpu9255_data gyro)
{
	float e_x, e_y, e_z;
	float g_x, g_y, g_z;
	float norm;

	norm = sqrt(a_y*a_y + a_x*a_x + a_z*a_z);
	a_x = a_x / norm;
	a_y = a_y / norm;
	a_z = a_z / norm; 

	g_x = 2 * (x*z - s*y);
	g_y = 2 * (y*z + s*x);
	g_z = 1 - 2 * (x*x + y*y);   

	e_x = a_y*g_z - a_z*g_y;
	e_y = a_z*g_x - a_x*g_z;
	e_z = a_x*g_y - a_y*g_x;

	eInt_x = eInt_x + e_x*Ki;
	eInt_y = eInt_y + e_y*Ki;
	eInt_z = eInt_z + e_z*Ki;

	w[0] = w_x + Kp*e_x + eInt_x;
	w[1] = w_y + Kp*e_y + eInt_y;
	w[2] = w_z + Kp*e_z + eInt_z;
}
/*
//卡尔曼滤波 _(:з」∠)_
void KF(){
	
}
*/

//PID部分--------------------------------------------------

typedef struct{
	float kp, ki, kd;
	float err, err1, err2;
	float errint;
}PIDparameter;

PIDparameter pid_data[] = {
//   kp  ki  kd
	{0.8, 0, 0, 0, 0, 0, 0}, //内环pitch
	{0.8, 0, 0, 0, 0, 0, 0}, //内环roll
	{0.8, 0, 0, 0, 0, 0, 0}, //内环yaw
	{0.8, 0, 0, 0, 0, 0, 0}, //外环机体坐标x方向角速度
	{0.8, 0, 0, 0, 0, 0, 0}, //外环机体坐标y方向角速度
	{0.8, 0, 0, 0, 0, 0, 0}  //外环机体坐标z方向角速度
};

float PID(float set, float actual, PIDparameter haha) {
    haha.err2 = haha.err1;
	haha.err1 = haha.err;
    
	haha.err = set - actual;
	haha.errint += haha.err;
	
	return haha.kp*haha.err + haha.ki*haha.errint + haha.kd*(3 * haha.err - 4 * haha.err1 + haha.err2);
}


float pid_out[3]={0,0,0};

void pid_motor(float *set, float *actual, PIDparameter pid_value[]){
    unsigned char i=0;
    for (i=0; i<3; i++){
        pid_out[i] = PID(PID(set[i], actual[i], pid_value[i]),
                         w[i], pid_value[i+3]);
    }
    return pid_out;
}


float *AHRS(mpu9255_data accel, mpu9255_data gyro, PIDparameter pid_QAQ, float th_set[], float dt){
    float th[3];
    COM_FILT(accel, gyro);
    updata(dt);
    QtoE();
    
    th[0] = pitch;
    th[1] = roll;
    th[0] = yaw;
    
    return pid_out(th_set, th, pid_QAQ);
}





