#ifndef __PWM_H__
#define __PWM_H__


#define PWMOUTP1 0x08           //切换到P1
#define PWMOUTP2 0x00           //切换到P2
#define PWMOUTP6 0x10           //切换到P6

void SetPWMf(unsigned int f);                                                            //设置PWM频率，单位Hz
void SetPWMpd(unsigned char n, unsigned int phase, unsigned char duty_cycle);            //设置PWM占空比和相位，n为PWM编号, 占空比0到1000
void PWMswich(unsigned char i);                                                          //开启PWM，i的每一位控制一个PWM，高电平有效，如0x21第零个和第5个PWM有效
void PWM_outP(unsigned char i, unsigned char m);                          //设置PWM输出管脚，i的每一位选中一个PWM，m为PWMOUTP1,PWMOUTP2,PWMOUTP6三项
                                                                          //若让PWM5和PWM0切换到P6则PWMoutP(0x21, PWMOUTP6)
#endif