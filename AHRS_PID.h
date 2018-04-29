typedef struct mpu9255_data_struct{
    float x;
    float y;
    float z;
}mpu9255_data;

mpu9255_data *accel;
mpu9255_data *gyro;
mpu9255_data *mag;

void init_AHRS(float dt, float gx, float gy, float gz, float dthx, float dthy, float dthz);
//获取初始四元数,第一个参数为时基

float *AHRS(float *pwm, mpu9255_data accel, mpu9255_data gyro, float nrf[], PIDparameter pid_QAQ, float dt);
