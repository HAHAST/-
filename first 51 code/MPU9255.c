#include  "STC15F2K60S2.H"
#include  "INTRINS.H"
#include  "MPU9255.H"
#include  "STDLIB.H"
#include  "USEFUL.H"

sbit SCL =P3^0;
sbit SDA =P3^1;

bit busy;

#define CONFIG              				0x1A

//**************************************
//I2C起始信号
//**************************************
void I2C_Start()
{
    SDA = 1;                    //拉高数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 0;                    //产生下降沿
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
}
//**************************************
//I2C停止信号
//**************************************
void I2C_Stop()
{
    SDA = 0;                    //拉低数据线
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SDA = 1;                    //产生上升沿
    Delay5us();                 //延时
}
//**************************************
//I2C发送应答信号
//入口参数:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(bit ack)
{
    SDA = ack;                  //写应答信号
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
}
//**************************************
//I2C接收应答信号
//**************************************
bit I2C_RecvACK()
{
    SCL = 1;                    //拉高时钟线
    Delay5us();                 //延时
    CY = SDA;                   //读应答信号
    SCL = 0;                    //拉低时钟线
    Delay5us();                 //延时
    return CY;
}
//**************************************
//向I2C总线发送一个字节数据
//**************************************
void I2C_SendByte(unsigned char dat)
{
    unsigned char i;
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;              //移出数据的最高位
        SDA = CY;               //送数据口
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    I2C_RecvACK();
}
//**************************************
//从I2C总线接收一个字节数据
//**************************************
unsigned char I2C_RecvByte()
{
    unsigned char i;
    unsigned char dat = 0;
    SDA = 1;                    //使能内部上拉,准备读取数据,
    for (i=0; i<8; i++)         //8位计数器
    {
        dat <<= 1;
        SCL = 1;                //拉高时钟线
        Delay5us();             //延时
        dat |= SDA;             //读数据               
        SCL = 0;                //拉低时钟线
        Delay5us();             //延时
    }
    return dat;
}
//**************************************
//向I2C设备写入一个字节数据
//**************************************
void Single_WriteI2C(unsigned char _I2C_ADDR,unsigned char REG_Address,unsigned char REG_data)
{
    I2C_Start();                  //起始信号
    I2C_SendByte(_I2C_ADDR);   //发送设备地址+写信号
    I2C_SendByte(REG_Address);    //内部寄存器地址，
    I2C_SendByte(REG_data);       //内部寄存器数据，
    I2C_Stop();                   //发送停止信号
}
//**************************************
//从I2C设备读取一个字节数据
//**************************************
unsigned char Single_ReadI2C(unsigned char _I2C_ADDR,unsigned char REG_Address)
{
	unsigned char REG_data;
	I2C_Start();                   //起始信号
	I2C_SendByte(_I2C_ADDR);    //发送设备地址+写信号
	I2C_SendByte(REG_Address);     //发送存储单元地址，从0开始	
	I2C_Start();                   //起始信号
	I2C_SendByte(I2C_ADDR + 1);  //发送设备地址+读信号
	REG_data=I2C_RecvByte();       //读出寄存器数据
	I2C_SendACK(1);                //接收应答信号
	I2C_Stop();                    //停止信号
	return REG_data;
}
int GetData(unsigned char REG_Address)
{
	char H,L;
	H = Single_ReadI2C(I2C_ADDR,REG_Address);
	L = Single_ReadI2C(I2C_ADDR,REG_Address+1);
	return (H<<8)+L;//合成数据
}
#ifdef USE_MAG
int GetData_Mag(unsigned char _mag)
{
	char H,L;
	if(_mag == 0)
	{
		H = Single_ReadI2C(I2C_ADDR,EXT_SENS_DATA_00);
		L = Single_ReadI2C(I2C_ADDR,EXT_SENS_DATA_01);
	}
	else if(_mag == 1)
	{
		H = Single_ReadI2C(I2C_ADDR,EXT_SENS_DATA_02);
		L = Single_ReadI2C(I2C_ADDR,EXT_SENS_DATA_03);
	}
	else if(_mag == 2)
	{
		H = Single_ReadI2C(I2C_ADDR,EXT_SENS_DATA_04);
		L = Single_ReadI2C(I2C_ADDR,EXT_SENS_DATA_05);
	}
	return (H << 8) + L;
}
void INIT_MAG()
{
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_ADDR,0x0C);
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_REG,0x0A);
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_CTRL,0x81);
	Single_WriteI2C(I2C_ADDR,USER_CTRL, 0x20);
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_DO, 0x16);
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_ADDR,0x8C);
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_REG,0x03);
	Single_WriteI2C(I2C_ADDR,I2C_SLV0_CTRL,0x86);
}
#endif
void MPU9255_CONFIGURATION()
{
	Single_WriteI2C(I2C_ADDR,PWR_MGMT_1,0x00);
	Single_WriteI2C(I2C_ADDR,SMPLRT_DIV, 0x07);
	Single_WriteI2C(I2C_ADDR,CONFIG, 0x04);
	Single_WriteI2C(I2C_ADDR,GYRO_CONFIG, 0x08);
	Single_WriteI2C(I2C_ADDR,ACCEL_CONFIG, 0x08);
	Single_WriteI2C(I2C_ADDR,ACCEL_CONFIG_2,0x03);
}