#include  "STC15F2K60S2.H"
#include  "INTRINS.H"
#include  "NRF24L01.H"

//******************************************************************************************
uchar bdata sta;   //状态标志
sbit RX_DR  = sta ^ 6;
sbit TX_DS  = sta ^ 5;
sbit MAX_RT = sta ^ 4;
uchar code TX_ADDRESS[TX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; //本地地址
uchar code RX_ADDRESS[RX_ADR_WIDTH] = {0x34, 0x43, 0x10, 0x10, 0x01}; //接收地址
/******************************************延时函数********************************************************/
//短延时
void delay_ms(unsigned int x)//@24.000MHz
{
	unsigned char i, j;

	for(;x > 0;x --)
	{
		i = 24;
		j = 85;
		do
		{
			while (--j);
		} while (--i);
	}
}
/************************************IO 口模拟SPI总线 代码************************************************/
uchar SPI_RW(uchar byte)
{
    uchar bit_ctr;
    for(bit_ctr = 0; bit_ctr < 8; bit_ctr++)
    {
        MOSI = (byte & 0x80);

        byte = (byte << 1);
        SCK = 1;
        byte |= MISO;
        //led=MISO;Delay(150);
        SCK = 0;
    }
    return(byte);
}
uchar SPI_RW_Reg  (uchar  reg, uchar value) // 向寄存器REG写一个字节，同时返回状态字节
{
    uchar status;
    CSN = 0;
    status = SPI_RW(reg);
    SPI_RW(value);
    CSN = 1;
    return(status);
}
uchar SPI_Read (uchar  reg )
{
    uchar reg_val;
    CSN = 0;
    SPI_RW(reg);
    reg_val = SPI_RW(0);
    CSN = 1;
    return(reg_val);
}
uchar SPI_Write_Buf(uchar reg, uchar *pBuf, uchar bytes)
{
    uchar status, byte_ctr;
    CSN = 0;                   // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);    // Select register to write to and read status byte
    for(byte_ctr = 0; byte_ctr < bytes; byte_ctr++) // then write all byte in buffer(*pBuf)
        SPI_RW(*pBuf++);
    CSN = 1;                 // Set CSN high again
    return(status);          // return nRF24L01 status byte
}
#if MODE
/*******************************发*****送*****模*****式*****代*****码*************************************/
void TX_Mode(void)
{
    CE = 0;

    SPI_RW_Reg(FLUSH_TX, 0x00);
    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack
    SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
    SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
    SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...1a
    SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
    SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
    SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为2字节
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);
    CE = 1;
    delay_ms(100);
}
void Transmit(unsigned char *tx_buf)
{
    CE = 0; //StandBy I模式
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // 装载接收端地址
    SPI_RW_Reg(FLUSH_TX, 0x00);
    SPI_Write_Buf(WR_TX_PLOAD, tx_buf, TX_PLOAD_WIDTH);     // 装载数据
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0e);      // IRQ收发完成中断响应，16位CRC，主发送
    CE = 1; //置高CE，激发数据发送
    delay_ms(150);
}
#else
/*******************************接*****收*****模*****式*****代*****码*************************************/
uchar SPI_Read_Buf(uchar reg, uchar *pBuf, uchar uchars)
{
    uchar status, uchar_ctr;

    CSN = 0;                      // Set CSN low, init SPI tranaction
    status = SPI_RW(reg);         // Select register to write to and read status uchar

    for(uchar_ctr = 0; uchar_ctr < uchars; uchar_ctr++)
        pBuf[uchar_ctr] = SPI_RW(0);    //

    CSN = 1;
    return(status);                    // return nRF24L01 status uchar
}
/******************************************************************************************************/
/*函数：unsigned char nRF24L01_RxPacket(unsigned char* rx_buf)
/*功能：数据读取后放如rx_buf接收缓冲区中
/******************************************************************************************************/
unsigned char nRF24L01_RxPacket(unsigned char *rx_buf)
{
    unsigned char revale = 0;
    sta = SPI_Read(STATUS); // 读取状态寄存其来判断数据接收状况
    if(RX_DR)    // 判断是否接收到数据
    {
        //CE = 0;    //SPI使能
        SPI_Read_Buf(RD_RX_PLOAD, rx_buf, RX_PLOAD_WIDTH); // read receive payload from RX_FIFO buffer
        revale = 1;  //读取数据完成标志
        //Delay(100);
    }
    SPI_RW_Reg(WRITE_REG + STATUS, sta); //接收到数据后RX_DR,TX_DS,MAX_PT都置高为1，通过写1来清楚中断标志
    return revale;
}
/****************************************************************************************************/
/*函数：void RX_Mode(void)
/*功能：数据接收配置
/****************************************************************************************************/
void RX_Mode(void)
{
    CE = 0;

    SPI_RW_Reg(FLUSH_RX, 0x00);
    SPI_Write_Buf(WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);    // Writes TX_Address to nRF24L01
    SPI_Write_Buf(WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH); // RX_Addr0 same as TX_Adr for Auto.Ack

    SPI_RW_Reg(WRITE_REG + EN_AA, 0x01);      // Enable Auto.Ack:Pipe0
    SPI_RW_Reg(WRITE_REG + EN_RXADDR, 0x01);  // Enable Pipe0
    SPI_RW_Reg(WRITE_REG + SETUP_RETR, 0x1a); // 500us + 86us, 10 retrans...1a
    SPI_RW_Reg(WRITE_REG + RF_CH, 40);        // Select RF channel 40
    SPI_RW_Reg(WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH); //设置接收数据长度，本次设置为5字节
    SPI_RW_Reg(WRITE_REG + RF_SETUP, 0x07);   // TX_PWR:0dBm, Datarate:1Mbps, LNA:HCURR
    SPI_RW_Reg(WRITE_REG + CONFIG, 0x0F);
    CE = 1;
    delay_ms(130);
}
#endif