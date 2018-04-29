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
																																								
																																								
�ռ�����							��ַ��Χ							˵��																
																																								
																																								
DATA									D:00H~7FH							Ƭ��RAMֱ��Ѱַ��										
																																								
BDATA									D"20H~2FH							Ƭ��RAMλѰַ��

IDATA									I:00H~FFH							Ƭ��RAM���Ѱַ��

XDATA									X:0000H~FFFFH					64KB����Ƭ��RAM������

HDATA									X:0000H~FFFFFFH				16MB��չƬ��RAM������

CODE									C:0000H~FFFFH					64K����Ƭ����ROM������

HCONST(ECODE)					C:0000H~FFFFFFH				16MB��չƬ��ROM������

BANK0~BANK31					B0:0000H~FFFFH				�����������������չ32X64KB ROM
											:	
											:
											B31:0000H~FFFFH
-------------------------------------------------------------------------------*/
/*************�����ļ�************/
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

void GET_INIT_Q()  //��ȡ��ʼ��Ԫ��
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
unsigned char Rx_9 = 0;  //���ڼ�����ݱ仯
unsigned char RX_DATA_CONV;  //�������ݵȼ�
unsigned char STAS_DIG = 0;  //����100ʱһ��û�յ������ź�




float xdata thu;  //����
//

/********************************************************************/



/*******************************MAIN����*****************************/

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
	unsigned char THU_LVL,FB_LVL,LR_LVL;  //����ǰ�����ҷּ�
	
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
		
		nRF24L01_RxPacket(Rx_Buf);		//������������
		
		THU_LVL = Rx_Buf[0] / 25;  //���ŷּ�
		FB_LVL = 10 - (Rx_Buf[1] / 25);  //ǰ��ּ�
		LR_LVL = Rx_Buf[2] / 25;  //���ҷּ�
		
		P0 = Rx_Buf[0];  //ͬ��
		thu = Rx_Buf[0];
//		a = ((255 - Rx_Buf[1]) - 127) * 0.0784313725;
//		b = (Rx_Buf[2] - 127) * 0.0784313725;
		
		//MOTOR_CTRL(thu, PID(a, AHRS(GETGX(), GETGY(), GETGZ(), w_x, w_y, w_z), w_x, w_y));
		MPA(thu, a, w_x, w_y, w_z);
			
		while(!TF0);
		TF0 = 0;
	}
}