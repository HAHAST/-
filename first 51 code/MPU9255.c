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
//I2C��ʼ�ź�
//**************************************
void I2C_Start()
{
    SDA = 1;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 0;                    //�����½���
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
}
//**************************************
//I2Cֹͣ�ź�
//**************************************
void I2C_Stop()
{
    SDA = 0;                    //����������
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SDA = 1;                    //����������
    Delay5us();                 //��ʱ
}
//**************************************
//I2C����Ӧ���ź�
//��ڲ���:ack (0:ACK 1:NAK)
//**************************************
void I2C_SendACK(bit ack)
{
    SDA = ack;                  //дӦ���ź�
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
}
//**************************************
//I2C����Ӧ���ź�
//**************************************
bit I2C_RecvACK()
{
    SCL = 1;                    //����ʱ����
    Delay5us();                 //��ʱ
    CY = SDA;                   //��Ӧ���ź�
    SCL = 0;                    //����ʱ����
    Delay5us();                 //��ʱ
    return CY;
}
//**************************************
//��I2C���߷���һ���ֽ�����
//**************************************
void I2C_SendByte(unsigned char dat)
{
    unsigned char i;
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;              //�Ƴ����ݵ����λ
        SDA = CY;               //�����ݿ�
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    I2C_RecvACK();
}
//**************************************
//��I2C���߽���һ���ֽ�����
//**************************************
unsigned char I2C_RecvByte()
{
    unsigned char i;
    unsigned char dat = 0;
    SDA = 1;                    //ʹ���ڲ�����,׼����ȡ����,
    for (i=0; i<8; i++)         //8λ������
    {
        dat <<= 1;
        SCL = 1;                //����ʱ����
        Delay5us();             //��ʱ
        dat |= SDA;             //������               
        SCL = 0;                //����ʱ����
        Delay5us();             //��ʱ
    }
    return dat;
}
//**************************************
//��I2C�豸д��һ���ֽ�����
//**************************************
void Single_WriteI2C(unsigned char _I2C_ADDR,unsigned char REG_Address,unsigned char REG_data)
{
    I2C_Start();                  //��ʼ�ź�
    I2C_SendByte(_I2C_ADDR);   //�����豸��ַ+д�ź�
    I2C_SendByte(REG_Address);    //�ڲ��Ĵ�����ַ��
    I2C_SendByte(REG_data);       //�ڲ��Ĵ������ݣ�
    I2C_Stop();                   //����ֹͣ�ź�
}
//**************************************
//��I2C�豸��ȡһ���ֽ�����
//**************************************
unsigned char Single_ReadI2C(unsigned char _I2C_ADDR,unsigned char REG_Address)
{
	unsigned char REG_data;
	I2C_Start();                   //��ʼ�ź�
	I2C_SendByte(_I2C_ADDR);    //�����豸��ַ+д�ź�
	I2C_SendByte(REG_Address);     //���ʹ洢��Ԫ��ַ����0��ʼ	
	I2C_Start();                   //��ʼ�ź�
	I2C_SendByte(I2C_ADDR + 1);  //�����豸��ַ+���ź�
	REG_data=I2C_RecvByte();       //�����Ĵ�������
	I2C_SendACK(1);                //����Ӧ���ź�
	I2C_Stop();                    //ֹͣ�ź�
	return REG_data;
}
int GetData(unsigned char REG_Address)
{
	char H,L;
	H = Single_ReadI2C(I2C_ADDR,REG_Address);
	L = Single_ReadI2C(I2C_ADDR,REG_Address+1);
	return (H<<8)+L;//�ϳ�����
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