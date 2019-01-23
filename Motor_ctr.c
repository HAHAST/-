


void ctr_motor(float *pid_out, float thu, void (*motor_power)(unsigned char, unsigned char, unsigned char, unsigned char) reentrant){
    unsigned char motor_1, motor_2, motor_3, motor_4;
    
    motor_1 = thu + pid_out[2] + pid_out[0] + pid_out[1];
    motor_2 = thu - pid_out[2] + pid_out[0] - pid_out[1];
    motor_3 = thu + pid_out[2] - pid_out[0] - pid_out[1];
    motor_4 = thu - pid_out[2] - pid_out[0] + pid_out[1];
    
    motor_power(motor_1, motor_2, motor_3, motor_4);
}























