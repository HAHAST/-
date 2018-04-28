#include <math.h>
#include "AHRS_PID.h"

void EtoQ(float *s, float *x, float *y, float *z, float pitch, float roll, float yaw)  //欧拉角转四元数
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
void QtoE(float *pitch, float *roll, float *yaw, float s, float x, float y, float z){
	pitch = asin(2 * (z*y + s * x));
	yaw = atan2(2 * (z*s - y * x), (1 - 2 * x*x - 2 * z*z));
	roll = atan2(2 * (s*y - x * z), (1 - 2 * y*y - 2 * x*x));
}


//姿态更新
void updata(mpu9255_data dth, float dt){
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

/*
//卡尔曼滤波 _(:з」∠)_
void KF(){
	
}
*/

//PID部分

float PID(float set, float actual, unsigned char pid_i) {
    haha[pid_i].err2 = haha[pid_i].err1;
	haha[pid_i].err1 = haha[pid_i].err;
    
	haha[pid_i].err = set - actual;
	haha[pid_i].errint += haha[pid_i].err;
	
	return haha[i].kp*haha[i].err + haha[i].ki*haha[i].errint + haha[i].kd*(3 * haha[i].err - 4 * haha[i].err1 + haha[i].err2);
}











