/*
        quu..__
         $$$b  `---.__
          "$$b        `--.                          ___.---uuudP
           `$$b           `.__.------.__     __.---'      $$$$"              .
             "$b          -'            `-.-'            $$$"              .'|
               ".                                       d$"             _.'  |
                 `.   /                              ..."             .'     |
                   `./                           ..::-'            _.'       |
                    /                         .:::-'            .-'         .'
                   :                          ::''\          _.'            |
                  .' .-.             .-.           `.      .'               |
                  : /'$$|           .@"$\           `.   .'              _.-'
                 .'|$u$$|          |$$,$$|           |  <            _.-'
                 | `:$$:'          :$$$$$:           `.  `.       .-'
                 :                  `"--'             |    `-.     \
                :##.       ==             .###.       `.      `.    `\
                |##:                      :###:        |        >     >
                |#'     `..'`..'          `###'        x:      /     /
                 \                                   xXX|     /    ./
                  \                                xXXX'|    /   ./
                  /`-.                                  `.  /   /
                 :    `-  ...........,                   | /  .'
                 |         ``:::::::'       .            |<    `.
                 |             ```          |           x| \ `.:``.
                 |                         .'    /'   xXX|  `:`M`M':.
                 |    |                    ;    /:' xXXX'|  -'MMMMM:'
                 `.  .'                   :    /:'       |-'MMMM.-'
                  |  |                   .'   /'        .'MMM.-'
                  `'`'                   :  ,'          |MMM<
                    |                     `'            |tbap\
                     \                                  :MM.-'
                      \                 |              .''
                       \.               `.            /
                        /     .:::::::.. :           /
                       |     .:::::::::::`.         /
                       |   .:::------------\       /
                      /   .''               >::'  /
                      `',:                 :    .'
                                           `:.:'

         
*/
/*------------------------------------------------------------------------------
																																								
																																								
空间名称							地址范围							说明																
																																								
																																								
DATA									D:00H~7FH							片内RAM直接寻址区										
																																								
BDATA									D"20H~2FH							片内RAM位寻址区

IDATA									I:00H~FFH							片内RAM间接寻址区

XDATA									X:0000H~FFFFH					64KB常规片外RAM数据区

HDATA									X:0000H~FFFFFFH				16MB拓展片外RAM数据区

CODE									C:0000H~FFFFH					64K常规片内外ROM代码区

HCONST(ECODE)					C:0000H~FFFFFFH				16MB拓展片外ROM常数区

BANK0~BANK31					B0:0000H~FFFFH				分组代码区，最大可拓展32X64KB ROM
											:	
											:
											B31:0000H~FFFFH
-------------------------------------------------------------------------------*/
/*************包含文件************/
#include  "STC15F2K60S2.H"         
#include  "INTRINS.H"
#include  "USEFUL.H"
#include  "PWM.H"
#include  "NRF24L01.H"
#include  "MATH.H"
//#include  "PID_IMU.H"
#include  "MPU9255.H"

/***********************************/

#define Ki 	 0.001f
#define Kp    0.8f
#define T 		0.01f


/********************************************************************/
bit FLAG = 0;
unsigned char Rx_Buf[RX_PLOAD_WIDTH];
unsigned char Rx_9 = 0;  //用于检测数据变化
unsigned char RX_DATA_CONV;  //油门数据等级
unsigned char STAS_DIG = 0;  //等于100时一秒没收到无线信号

float s = 1, x = 0, y = 0, z = 0;  //四元数 
float pitch = 0, yaw = 0, roll = 0; 
float w_x0 = 0, w_y0 = 0, w_z0 = 0;                     
float eInt_x = 0, eInt_y = 0, eInt_z = 0;  //互补滤波误差积分

typedef struct
{
	float kp, ki, kd;
	float err, err1, err2;
	float errint;
}PIDparameter;

PIDparameter haha[] = {
	{0.8,0,0,0,0,0,0},
	{0.8,0,0,0,0,0,0},
	{0.8,0,0,0,0,0,0},
	{0.8,0,0,0,0,0,0},
	{0.8,0,0,0,0,0,0},
	{0.8,0,0,0,0,0,0}
};

float PID_yaw = 0, PID_pitch = 0, PID_roll = 0;
float PIDoutr0 = 0, PIDoutr1 = 0, PIDoutr2 = 0;
float w_x, w_y, w_z;

float xdata Motor1, Motor2, Motor3, Motor4;
float xdata thu;  //油门
//
float PID(float set, float actual, unsigned char i) {
	float u;

	haha[i].err = set - actual;
	haha[i].errint += haha[i].err;
	u = haha[i].kp*haha[i].err + haha[i].ki*haha[i].errint + haha[i].kd*(3 * haha[i].err - 4 * haha[i].err1 + haha[i].err2);

	haha[i].err2 = haha[i].err1;
	haha[i].err1 = haha[i].err;

	return u;
}
/********************************************************************/
void Q_E() //四元数转欧拉角
{
	pitch = asin(2 * (z*y + s*x));
	yaw = atan2(2 * (z*s - y*x), (1 - 2 * x*x - 2 * z*z));
	roll = atan2(2 * (s*y - x*z), (1 - 2 * y*y - 2 * x*x));
}
//
void E_Q()  //欧拉角转四元数
{
	float norm;

	s = cos(roll / 2)*cos(pitch / 2)*cos(yaw / 2) - sin(yaw / 2)*sin(roll / 2)*sin(pitch / 2);
	x = cos(roll / 2)*sin(pitch / 2)*cos(yaw / 2) - sin(roll / 2)*cos(pitch / 2)*sin(yaw / 2);
	y = cos(roll / 2)*sin(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*cos(pitch / 2)*cos(yaw / 2);
	z = cos(roll / 2)*cos(pitch / 2)*sin(yaw / 2) + sin(roll / 2)*sin(pitch / 2)*cos(yaw / 2);

	norm = sqrt(s*s+x*x+y*y+z*z);
	s = s / norm;
	x = x / norm;
	y = y / norm;
	z = z / norm;   
}
void POS_UPDATE(float w[3])  //姿态更新
{
	float cup0, cup1, cup2, cup3, norm;
	cup0 = s - 0.5*(w[0]*x + w[1]*y + w[2]*z)*T;
	cup1 = x + 0.5*(w[0]*s + w[2]*y - w[1]*z)*T;
	cup2 = y + 0.5*(w[1]*s - w[2]*x + w[0]*z)*T;
	cup3 = z + 0.5*(w[2]*s + w[1]*x - w[0]*y)*T;
	norm = sqrt(cup0*cup0 + cup1*cup1 + cup2*cup2 + cup3*cup3);
	s = cup0 / norm;
	x = cup1 / norm;
	y = cup2 / norm;
	z = cup3 / norm;
}
//
void COM_FILT(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z, float w[3])  //互补滤波
{
	float e_x, e_y, e_z;
	float g_x, g_y, g_z;
	float norm;

	norm = sqrt(a_y*a_y + a_x*a_x + a_z*a_z);
	a_x = a_x / norm;
	a_y = a_y / norm;
	a_z = a_z / norm; 

	g_x = 2 * (x*z - s*y);
	g_y = 2 * (y*z + s*x);
	g_z = 1 - 2 * (x*x + y*y);   

	e_x = a_y*g_z - a_z*g_y;
	e_y = a_z*g_x - a_x*g_z;
	e_z = a_x*g_y - a_y*g_x;

	eInt_x = eInt_x + e_x*Ki;
	eInt_y = eInt_y + e_y*Ki;
	eInt_z = eInt_z + e_z*Ki;

	w[0] = w_x + Kp*e_x + eInt_x;
	w[1] = w_y + Kp*e_y + eInt_y;
	w[2] = w_z + Kp*e_z + eInt_z;
}
//
void GET_INIT_Q()  //获取初始四元数
{
	unsigned char i;
	float g_x = 0, g_y = 0, g_z = 0;
	
	for (i = 0; i < 10; i++) {
		g_x += GetData(ACCEL_XOUT_H) / 8192;
		g_y += GetData(ACCEL_YOUT_H) / 8192;
		g_z += GetData(ACCEL_ZOUT_H) / 8192;
	}

	g_x = 0.1*g_x;
	g_y = 0.1*g_y;
	g_z = 0.1*g_z;
	
	pitch = atan2(g_y, g_z);
	roll = atan2(g_x, g_z);
	E_Q();
}
void MOTOR_CTRL(float PID_yaw, float PID_pitch, float PID_roll)  //PID控制值加载至电机
{
	Motor1 = thu + PID_yaw + PID_pitch + PID_roll;
	Motor2 = thu - PID_yaw + PID_pitch - PID_roll;
	Motor3 = thu + PID_yaw - PID_pitch - PID_roll;
	Motor4 = thu - PID_yaw - PID_pitch + PID_roll;
	
	Motor1 = Motor1 * 7.843;
	Motor2 = Motor2 * 7.843;
	Motor3 = Motor3 * 7.843;
	Motor4 = Motor4 * 7.843;
	
	if (Motor1 > 2000) Motor1 = 2000;
  if (Motor2 > 2000) Motor2 = 2000;
	if (Motor3 > 2000) Motor3 = 2000;
	if (Motor4 > 2000) Motor4 = 2000;
	
	if (Motor1 < 0) Motor1 = 0;
  if (Motor2 < 0) Motor2 = 0;
	if (Motor3 < 0) Motor3 = 0;
	if (Motor4 < 0) Motor4 = 0;
	
	PWM2T2 = 2000 + (unsigned int)Motor1;
	PWM3T2 = 2000 + (unsigned int)Motor2;
	PWM4T2 = 2000 + (unsigned int)Motor3;
	PWM5T2 = 2000 + (unsigned int)Motor4;
	
	//PWM2T2 = 2000;
	
	//_PWM_(Motor1, Motor2, Motor3, Motor4);
}
void AHRS(float a_x, float a_y, float a_z, float w_x, float w_y, float w_z)
{
	float w[3];
	
	COM_FILT(a_x, a_y, a_z, w_x, w_y, w_z, w);
	
	POS_UPDATE(w);
	
	Q_E();
}
/*******************************MAIN函数*****************************/

//                      d*##$.
// zP"""""$e.           $"    $o
//4$       '$          $"      $
//'$        '$        J$       $F
// 'b        $k       $>       $
//  $k        $r     J$       d$
//  '$         $     $"       $~
//   '$        "$   '$E       $
//    $         $L   $"      $F ...
//     $.       4B   $      $$$*"""*b
//     '$        $.  $$     $$      $F
//      "$       R$  $F     $"      $
//       $k      ?$ u*     dF      .$
//       ^$.      $$"     z$      u$$$$e
//        #$b             $E.dW@e$"    ?$
//         #$           .o$$# d$$$$c    ?F
//          $      .d$$#" . zo$>   #$r .uF
//          $L .u$*"      $&$$$k   .$$d$$F
//           $$"            ""^"$$$P"$P9$
//          JP              .o$$$$u:$P $$
//          $          ..ue$"      ""  $"
//         d$          $F              $
//         $$     ....udE             4B
//          #$    """"` $r            @$
//           ^$L        '$            $F
//             RN        4N           $
//              *$b                  d$
//               $$k                 $F
//               $$b                $F
//                 $""               $F
//                 '$                $
//                  $L               $
//                  '$               $
//                   $               $
void main()
{
	float a = 0, b = 0, c = 0;
	unsigned char THU_LVL,FB_LVL,LR_LVL;  //油门前后左右分级
	
	IO_RESET();

	PWM_config();
	RX_Mode();
	Timer0Init();
	MPU9255_CONFIGURATION();
  E_Q();
	GET_INIT_Q();
	w_x0 = 0.0174533*GetData(GYRO_XOUT_H) / 65.5;
	w_y0 = 0.0174533*GetData(GYRO_XOUT_H) / 65.5;
	w_z0 = 0.0174533*GetData(GYRO_XOUT_H) / 65.5;
	
	P0 = SPI_Read(READ_REG +  STATUS);
	TR0 = 1;
	Delay1ms(5000);
	
	while (!nRF24L01_RxPacket(Rx_Buf));
	
	while(1)
	{
		w_x = 0.0174533*GetData(GYRO_XOUT_H) / 65.5 - w_x0;
		w_y = 0.0174533*GetData(GYRO_YOUT_H) / 65.5 - w_y0;
		w_z = 0.0174533*GetData(GYRO_ZOUT_H) / 65.5 - w_z0;
		
		AHRS(GetData(ACCEL_XOUT_H)/8192.0, GetData(ACCEL_YOUT_H)/8192.0, GetData(ACCEL_ZOUT_H)/8192.0, w_x, w_y, w_z);
		
		nRF24L01_RxPacket(Rx_Buf);		//接收无线数据
		
		THU_LVL = Rx_Buf[0] / 25;  //油门分级
		FB_LVL = 10 - (Rx_Buf[1] / 25);  //前后分级
		LR_LVL = Rx_Buf[2] / 25;  //左右分级
		
		P0 = Rx_Buf[0];  //同上
		thu = Rx_Buf[0];
		a = ((255 - Rx_Buf[1]) - 127) * 0.0784313725;
		b = (Rx_Buf[2] - 127) * 0.0784313725;
		
		PIDoutr0 = PID(0, 57.2957795 * yaw, 0); //PID
		PIDoutr1 = PID(a, 57.2957795 * pitch, 1);
		PIDoutr2 = PID(b, 57.2957795 * roll, 2);
		
		PID_yaw = PID(PIDoutr0, 57.2957795 * w_z, 3);
		PID_pitch = PID(PIDoutr1, 57.2957795 * w_x, 4);
		PID_roll = PID(PIDoutr2, 57.2957795 * w_y, 5);
		
		MOTOR_CTRL(PID_yaw, PID_pitch, PID_roll);
		
		
		//Delay1ms(10);
			
		while(!TF0);
		TF0 = 0;
	}
}
