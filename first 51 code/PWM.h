#ifndef __PWM_H__
#define __PWM_H__


#define PWMOUTP1 0x08           //�л���P1
#define PWMOUTP2 0x00           //�л���P2
#define PWMOUTP6 0x10           //�л���P6

void SetPWMf(unsigned int f);                                                            //����PWMƵ�ʣ���λHz
void SetPWMpd(unsigned char n, unsigned int phase, unsigned char duty_cycle);            //����PWMռ�ձȺ���λ��nΪPWM���, ռ�ձ�0��1000
void PWMswich(unsigned char i);                                                          //����PWM��i��ÿһλ����һ��PWM���ߵ�ƽ��Ч����0x21������͵�5��PWM��Ч
void PWM_outP(unsigned char i, unsigned char m);                          //����PWM����ܽţ�i��ÿһλѡ��һ��PWM��mΪPWMOUTP1,PWMOUTP2,PWMOUTP6����
                                                                          //����PWM5��PWM0�л���P6��PWMoutP(0x21, PWMOUTP6)
#endif