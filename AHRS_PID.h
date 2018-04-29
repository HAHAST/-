#ifndef __AHRS_PID_H__
#define __AHRS_PID_H__


typedef struct mpu9255_data_struct{
    float x;
    float y;
    float z;
}mpu9255_data;

typedef struct{
	float kp, ki, kd;
	float err, err1, err2;
	float errint;
}PIDparameter;


//获取初始四元数和陀螺仪静差,第一个参数传加速度，第二个参数传陀螺仪，第三个参数传定时器定时周期
void init_AHRS(mpu9255_data accel, mpu9255_data gyro, float time);


//姿态更新PID电机PWM， 第一个参数为输出4个元素的数组，分别为4个PWM的值
//第二个参数是加速度，第三个是陀螺仪，第四个是NRF的数组，第五个是PID的参数
//返回值和第一个参数一样，返回第一个参数的地址
float *AHRS(float *pwm, mpu9255_data accel, mpu9255_data gyro, float nrf[], PIDparameter pid_QAQ);




#endif
