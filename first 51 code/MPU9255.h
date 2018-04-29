#ifndef _MPU9255_H_
#define _MPU9255_H_

/*#ifndef _MPU6050_H_
	sbit    SCL=P3^0;			//IIC时钟引脚定义
	sbit    SDA=P3^1;			//IIC数据引脚定义
#endif*/

/* Exported types --------------------------------------------------------------------------*/
/*
|     |            ACCELEROMETER           |
| LPF |  BandW  |  Delay  | Noise | Sample |
+-----+---------+---------+-------+--------+
|  x  |  1130Hz |  0.75ms | 250Hz |  4kHz  |
|  0  |   460Hz |  1.94ms | 250Hz |  1kHz  |
|  1  |   184Hz |  5.80ms | 250Hz |  1kHz  |
|  2  |    92Hz |  7.80ms | 250Hz |  1kHz  |
|  3  |    41Hz | 11.80ms | 250Hz |  1kHz  |
|  4  |    20Hz | 19.80ms | 250Hz |  1kHz  |
|  5  |    10Hz | 35.70ms | 250Hz |  1kHz  |
|  6  |     5Hz | 66.96ms | 250Hz |  1kHz  |
|  7  |   460Hz |  1.94ms | 250Hz |  1kHz  |
*/
/*
|     |          GYROSCOPE         |    TEMPERATURE    |
| LPF |  BandW  |  Delay  | Sample |  BandW  |  Delay  |
+-----+---------+---------+--------+---------+---------+
|  x  |  8800Hz | 0.064ms |  32kHz |  4000Hz |  0.04ms |
|  x  |  3600Hz |  0.11ms |  32kHz |  4000Hz |  0.04ms |
|  0  |   250Hz |  0.97ms |   8kHz |  4000Hz |  0.04ms |
|  1  |   184Hz |   2.9ms |   1kHz |   188Hz |   1.9ms |
|  2  |    92Hz |   3.9ms |   1kHz |    98Hz |   2.8ms |
|  3  |    41Hz |   5.9ms |   1kHz |    42Hz |   4.8ms |
|  4  |    20Hz |   9.9ms |   1kHz |    20Hz |   8.3ms |
|  5  |    10Hz | 17.85ms |   1kHz |    10Hz |  13.4ms |
|  6  |     5Hz | 33.48ms |   1kHz |     5Hz |  18.6ms |
|  7  |  3600Hz |  0.17ms |   8kHz |  4000Hz |  0.04ms |
*/

//#define USE_MAG

#define X	0
#define Y 1
#define Z 2
/* Exported constants ----------------------------------------------------------------------*/

/* ---- MPU6555 Reg In MPU9255 ---------------------------------------------- */

#define I2C_ADDR            				0xD0   /* +1 Read    */
#define DeviceID            				0x73   /* In MPU9255 */

#define SELF_TEST_X_GYRO    				0x00
#define SELF_TEST_Y_GYRO    				0x01
#define SELF_TEST_Z_GYRO    				0x02
#define SELF_TEST_X_ACCEL   				0x0D
#define SELF_TEST_Y_ACCEL   				0x0E
#define SELF_TEST_Z_ACCEL   				0x0F
#define XG_OFFSET_H         				0x13
#define XG_OFFSET_L         				0x14
#define YG_OFFSET_H         				0x15
#define YG_OFFSET_L         				0x16
#define ZG_OFFSET_H         				0x17
#define ZG_OFFSET_L         				0x18
#define SMPLRT_DIV          				0x19

#define GYRO_CONFIG         				0x1B
#define ACCEL_CONFIG        				0x1C
#define ACCEL_CONFIG_2      				0x1D
#define LP_ACCEL_ODR        				0x1E
#define MOT_THR             				0x1F
#define FIFO_EN             				0x23
#define I2C_MST_CTRL        				0x24
#define I2C_SLV0_ADDR       				0x25
#define I2C_SLV0_REG        				0x26
#define I2C_SLV0_CTRL       				0x27
#define I2C_SLV1_ADDR       				0x28
#define I2C_SLV1_REG        				0x29
#define I2C_SLV1_CTRL       				0x2A
#define I2C_SLV2_ADDR       				0x2B
#define I2C_SLV2_REG        				0x2C
#define I2C_SLV2_CTRL       				0x2D
#define I2C_SLV3_ADDR       				0x2E
#define I2C_SLV3_REG        				0x2F
#define I2C_SLV3_CTRL       				0x30
#define I2C_SLV4_ADDR       				0x31
#define I2C_SLV4_REG        				0x32
#define I2C_SLV4_DO         				0x33
#define I2C_SLV4_CTRL       				0x34
#define I2C_SLV4_DI         				0x35
#define I2C_MST_STATUS      				0x36
#define INT_PIN_CFG         				0x37
#define INT_ENABLE          				0x38
#define INT_STATUS          				0x3A
#define ACCEL_XOUT_H        				0x3B
#define ACCEL_XOUT_L        				0x3C
#define ACCEL_YOUT_H        				0x3D
#define ACCEL_YOUT_L        				0x3E
#define ACCEL_ZOUT_H        				0x3F
#define ACCEL_ZOUT_L       				 	0x40
#define TEMP_OUT_H          				0x41
#define TEMP_OUT_L          				0x42
#define GYRO_XOUT_H         				0x43
#define GYRO_XOUT_L         				0x44
#define GYRO_YOUT_H         				0x45
#define GYRO_YOUT_L         				0x46
#define GYRO_ZOUT_H         				0x47
#define GYRO_ZOUT_L         				0x48
#define EXT_SENS_DATA_00    				0x49
#define EXT_SENS_DATA_01    				0x4A
#define EXT_SENS_DATA_02    				0x4B
#define EXT_SENS_DATA_03    				0x4C
#define EXT_SENS_DATA_04    				0x4D
#define EXT_SENS_DATA_05    				0x4E
#define EXT_SENS_DATA_06    				0x4F
#define EXT_SENS_DATA_07    				0x50
#define EXT_SENS_DATA_08    				0x51
#define EXT_SENS_DATA_09    				0x52
#define EXT_SENS_DATA_10    				0x53
#define EXT_SENS_DATA_11    				0x54
#define EXT_SENS_DATA_12    				0x55
#define EXT_SENS_DATA_13    				0x56
#define EXT_SENS_DATA_14    				0x57
#define EXT_SENS_DATA_15    				0x58
#define EXT_SENS_DATA_16    				0x59
#define EXT_SENS_DATA_17    				0x5A
#define EXT_SENS_DATA_18    				0x5B
#define EXT_SENS_DATA_19    				0x5C
#define EXT_SENS_DATA_20    				0x5D
#define EXT_SENS_DATA_21    				0x5E
#define EXT_SENS_DATA_22    				0x5F
#define EXT_SENS_DATA_23    				0x60
#define I2C_SLV0_DO         				0x63
#define I2C_SLV1_DO         				0x64
#define I2C_SLV2_DO         				0x65
#define I2C_SLV3_DO         				0x66
#define I2C_MST_DELAY_CTRL  				0x67
#define SIGNAL_PATH_RESET   				0x68
#define MOT_DETECT_CTRL     				0x69
#define USER_CTRL           				0x6A
#define PWR_MGMT_1          				0x6B
#define PWR_MGMT_2          				0x6C
#define FIFO_COUNTH         				0x72
#define FIFO_COUNTL         				0x73
#define FIFO_R_W            				0x74
#define WHO_AM_I            				0x75   /* ID = 0x73 In MPU9255 */
#define XA_OFFSET_H         				0x77
#define XA_OFFSET_L         				0x78
#define YA_OFFSET_H         				0x7A
#define YA_OFFSET_L         				0x7B
#define ZA_OFFSET_H         				0x7D
#define ZA_OFFSET_L         				0x7E

#define I2C_SLVx_EN         				0x80
#define I2C_SLV4_DONE       				0x40
#define I2C_SLV4_NACK       				0x10

/* ---- AK8963 Reg In MPU9255 ----------------------------------------------- */

#define AK8963_I2C_ADDR             0x0C
#define AK8963_DeviceID             0x48

/* Read-only Reg */
#define AK8963_WIA                  0x00
#define AK8963_INFO                 0x01
#define AK8963_ST1                  0x02
#define AK8963_HXL                  0x03
#define AK8963_HXH                  0x04
#define AK8963_HYL                  0x05
#define AK8963_HYH                  0x06
#define AK8963_HZL                  0x07
#define AK8963_HZH                  0x08
#define AK8963_ST2                  0x09
/* Write/Read Reg */
#define AK8963_CNTL1                0x0A
#define AK8963_CNTL2                0x0B
#define AK8963_ASTC                 0x0C
#define AK8963_TS1                  0x0D
#define AK8963_TS2                  0x0E
#define AK8963_I2CDIS               0x0F
/* Read-only Reg ( ROM ) */
#define AK8963_ASAX                 0x10
#define AK8963_ASAY                 0x11
#define AK8963_ASAZ                 0x12
/* Status */
#define AK8963_STATUS_DRDY          0x01
#define AK8963_STATUS_DOR           0x02
#define AK8963_STATUS_HOFL          0x08

void MPU9255_CONFIGURATION();
int GetData(unsigned char REG_Address);
unsigned char Single_ReadI2C(unsigned char _I2C_ADDR,unsigned char REG_Address);
void Single_WriteI2C(unsigned char _I2C_ADDR,unsigned char REG_Address,unsigned char REG_data);

#ifdef USE_MAG
int GetData_Mag(unsigned char _mag);
void INIT_MAG();
#endif

#endif