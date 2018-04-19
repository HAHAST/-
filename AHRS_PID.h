

void init_AHRS(float dt, float gx, float gy, float gz, float dthx, float dthy, float dthz);
//获取初始四元数,第一个参数为时基


void AHRS_getdata(unsigned char *set, float gx, float gy, float gz, float dthx, float dthy, float dthz);
//将MPU9255,遥控的数据传入AHRS，第一个参数传入NRF数组，2-4传入MPU的加速度，最后三个传入MPU的角速度



void PID_K(float *kp, float *ki, float *kd);
//传入PID的三个参数kp, ki, kd为数组，6个元素，分别为内环的和外环的


void AHRS_PID(void);
//进行姿态更新和PID



void PID_PWM(unsigned int *PWM1, unsigned int *PWM2, unsigned int *PWM3, unsigned int *PWM4);
//返回PID后四个PWM的值