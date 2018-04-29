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
#include  "NRF24L01.H"
#include  "MATH.H"
//#include  "PID_IMU.H"
#include  "MPU9255.H"
#include  "AHRS.H"
#include  "PID.H"
#include  "MOTOR.h" 

/***********************************/


#define GETGX() GetData(ACCEL_XOUT_H)/8192.0
#define GETGY() GetData(ACCEL_YOUT_H)/8192.0
#define GETGZ() GetData(ACCEL_ZOUT_H)/8192.0

#define MPA(t, l, n, m, k) MOTOR_CTRL(t, PID(l, AHRS(GETGX(), GETGY(), GETGZ(), n, m, k), n, m))

void GET_INIT_Q()  //获取初始四元数
{
	unsigned char i;
	float g_x = 0, g_y = 0, g_z = 0;
	
	for (i = 0; i < 10; i++) {
		g_x += GETGX();
		g_y += GETGY();
		g_z += GETGZ();
	}

	g_x = 0.1*g_x;
	g_y = 0.1*g_y;
	g_z = 0.1*g_z;
	
	init_Q(g_x, g_y, g_z);
}

/********************************************************************/
bit FLAG = 0;
unsigned char Rx_Buf[RX_PLOAD_WIDTH];
unsigned char Rx_9 = 0;  //用于检测数据变化
unsigned char RX_DATA_CONV;  //油门数据等级
unsigned char STAS_DIG = 0;  //等于100时一秒没收到无线信号




float xdata thu;  //油门
//

/********************************************************************/



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
	float w_x0, w_y0, w_z0;
	float w_x, w_y, w_z;
	float a[3] = {0};
	unsigned char THU_LVL,FB_LVL,LR_LVL;  //油门前后左右分级
	
	IO_RESET();

	MOTOR_init();
	RX_Mode();
	Timer0Init();
	MPU9255_CONFIGURATION();
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
		
		nRF24L01_RxPacket(Rx_Buf);		//接收无线数据
		
		THU_LVL = Rx_Buf[0] / 25;  //油门分级
		FB_LVL = 10 - (Rx_Buf[1] / 25);  //前后分级
		LR_LVL = Rx_Buf[2] / 25;  //左右分级
		
		P0 = Rx_Buf[0];  //同上
		thu = Rx_Buf[0];
//		a = ((255 - Rx_Buf[1]) - 127) * 0.0784313725;
//		b = (Rx_Buf[2] - 127) * 0.0784313725;
		
		//MOTOR_CTRL(thu, PID(a, AHRS(GETGX(), GETGY(), GETGZ(), w_x, w_y, w_z), w_x, w_y));
		MPA(thu, a, w_x, w_y, w_z);
			
		while(!TF0);
		TF0 = 0;
	}
}