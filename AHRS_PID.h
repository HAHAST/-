

void init_AHRS(float dt, float gx, float gy, float gz, float dthx, float dthy, float dthz);
//��ȡ��ʼ��Ԫ��,��һ������Ϊʱ��


void AHRS_getdata(unsigned char *set, float gx, float gy, float gz, float dthx, float dthy, float dthz);
//��MPU9255,ң�ص����ݴ���AHRS����һ����������NRF���飬2-4����MPU�ļ��ٶȣ������������MPU�Ľ��ٶ�



void PID_K(float *kp, float *ki, float *kd);
//����PID����������kp, ki, kdΪ���飬6��Ԫ�أ��ֱ�Ϊ�ڻ��ĺ��⻷��


void AHRS_PID(void);
//������̬���º�PID



void PID_PWM(unsigned int *PWM1, unsigned int *PWM2, unsigned int *PWM3, unsigned int *PWM4);
//����PID���ĸ�PWM��ֵ