//24MHz时钟

#include "STC8.H"
#include "INTRINS.H"

#define FOSC 24000000                                              //系统时钟
#define T 1000L                                                     //PWM计数次数
#define TM(x) (65536 - FOSC/x)



void Timer2Init(unsigned long int f_time)		//24.000MHz
{
	AUXR |= 0x04;		          //定时器时钟1T模式
	T2L = TM(f_time);		      //设置定时初值
	T2H = TM(f_time)>>8;		    //设置定时初值
	AUXR |= 0x10;		           //定时器2开始计时
}  

void SetPWMf(unsigned int f){             //f为频率，单位Hz

	Timer2Init(f*T);
	
	P_SW2 = 0x80;          
	PWMCKS = 0x10;
	PWMC = T;
	P_SW2 = 0x00;          
}

void SetPWMpd(unsigned char n, unsigned int phase, unsigned char duty_cycle){
	unsigned int hi;
	unsigned int lo;
	
	hi = phase/360.0*T;
	lo = (hi + T/1000L*duty_cycle)%T;
	
	P_SW2 = 0x80;
	if (0 == n){
		PWM0T2 = hi;
		PWM0T1 = lo;
	}else if (1 == n){            
		PWM1T2 = hi;
		PWM1T1 = lo;
	}else if (2 == n){            
		PWM2T2 = hi;
		PWM2T1 = lo;
	}else if (3 == n){          
		PWM3T2 = hi;
		PWM3T1 = lo;
	}else if (4 == n){           
		PWM4T2 = hi;
		PWM4T1 = lo;
	}else if (5 == n){           
		PWM5T2 = hi;
		PWM5T1 = lo;
	}else if (6 == n){            
		PWM6T2 = hi;
		PWM6T1 = lo;
	}else if (7 == n){          
		PWM7T2 = hi;
		PWM7T1 = lo;
	}
	P_SW2 = 0x00;
}

void PWM_outP(unsigned char i, unsigned char m){
	P_SW2 = 0x80;
	
	if (0x01 & i){
		PWM0CR &= 0xE7;
		PWM0CR |= m;
	}
	if (0x02 & i){
		PWM1CR &= 0xE7;
		PWM1CR |= m;
	}
	if (0x04 & i){
		PWM2CR &= 0xE7;
		PWM2CR |= m;
	}
	if (0x08 & i){
		PWM3CR &= 0xE7;
		PWM3CR |= m;
	}
	if (0x10 & i){
		PWM4CR &= 0xE7;
		PWM4CR |= m;
	}
	if (0x20 & i){
		PWM5CR &= 0xE7;
		PWM5CR |= m;
	}
	if (0x40 & i){
		PWM6CR &= 0xE7;
		PWM6CR |= m;
	}
	if (0x80 & i){
		PWM7CR &= 0xE7;
		PWM7CR |= m;
	}
	
	P_SW2 = 0x00;
}

void PWMswich(unsigned char i){
	if (!i)
		PWMCR &= 0x7F;
	
	P_SW2 = 0x80;
	
	if (0x01 & i)
		PWM0CR |= 0x80;
	else
		PWM0CR &= 0x7F;
	if (0x02 & i)
		PWM1CR |= 0x80;
	else
		PWM1CR &= 0x7F;
	if (0x04 & i)
		PWM2CR |= 0x80;
	else
		PWM2CR &= 0x7F;
	if (0x08 & i)
		PWM3CR |= 0x80;
	else
		PWM3CR &= 0x7F;
	if (0x10 & i)
		PWM4CR |= 0x80;
	else
		PWM4CR &= 0x7F;
	if (0x20 & i)
		PWM5CR |= 0x80;
	else
		PWM5CR &= 0x7F;
	if (0x40 & i)
		PWM6CR |= 0x80;
	else
		PWM6CR &= 0x7F;
	if (0x80 & i)
		PWM7CR |= 0x80;
	else
		PWM7CR &= 0x7F;
	
	P_SW2 = 0x00;
	
	if (i)
		PWMCR |= 0x80;
	
}
