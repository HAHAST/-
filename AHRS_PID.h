#ifndef __AHRS_PID_H__
#define __AHRS_PID_H__


typedef struct Mpu9255_data_struct{
    float x;
    float y;
    float z;
}Mpu9255_data;

typedef struct{
	float kp, ki, kd;
	float err, err1, err2;
	float errint;
}PIDparameter;

/*


//PID参数 这一段代码可放在想要想要放到的文件里
PIDparameter pid_data[] = {
//   kp  ki  kd
    {0.8, 0, 0, 0, 0, 0, 0}, //内环pitch
    {0.8, 0, 0, 0, 0, 0, 0}, //内环roll
    {0.8, 0, 0, 0, 0, 0, 0}, //内环yaw
    {1,   0, 0, 0, 0, 0, 0}, //外环机体坐标x方向角速度
    {1,   0, 0, 0, 0, 0, 0}, //外环机体坐标y方向角速度
    {1,   0, 0, 0, 0, 0, 0}  //外环机体坐标z方向角速度
};

*/




//获取初始四元数和陀螺仪静差,第一个参数传加速度，第二个参数传陀螺仪，第三个参数传定时器定时周期
void init_AHRS(Mpu9255_data accel, Mpu9255_data gyro, float time);


//姿态更新PID电机PWM， 第一个参数为输出4个元素的数组，分别为4个PWM的值
//第二个参数是加速度，第三个是陀螺仪，第四个是NRF的数组，第五个是PID的参数
//返回值和第一个参数一样，返回第一个参数的地址
float *AHRS(float *pwm, Mpu9255_data accel, Mpu9255_data gyro, float nrf[], PIDparameter pid_QAQ);

//              AHRS输出数组pwm[] 4个元素对应电机图
//
//
//              pwm[0]                               pwm[1]
//                                 y轴
//                                  |
//                                  |
//                               MPU9255—————— x轴  
//                                  
//
//
//
//              pwm[4]                               pwm[3]
//
//



#endif



