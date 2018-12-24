#ifndef _AhrsImu_h_
#define _AhrsImu_h_

    extern float q0,q1,q2,q3;
    extern float Ki;
    extern float twoKi,twoKp;
    extern unsigned short sampleFreq;
    
    void MahonyAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az);
    void MahonyAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
    void Mahony(float gx, float gy, float gz, float ax, float ay, float az, float dt);

#endif
