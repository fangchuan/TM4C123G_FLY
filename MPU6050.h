#ifndef _MPU6050_H
#define _MPU6050_H

/**********************************************************MPU6050引脚定义**************************************************/
#define  		SCL				GPIO_PIN_6			//PC6=SCL
#define     SDA				GPIO_PIN_7			//PC7=SDA

/**********************************************************MPU6050库函数宏定义**********************************************/
#define  		SYSCTL_PERIPH_MPU6050_PORT     	SYSCTL_PERIPH_GPIOC
#define  		SYSCTL_PERIPH_MPU6050_I2C     	SYSCTL_PERIPH_I2C0
#define 	  MPU6050_PORT_BASE								GPIO_PORTC_BASE
#define 		MPU6050_I2C_BASE								I2C0_BASE
#define 		MPU6050_SCL											GPIO_PC6_I2C0SCL
#define 		MPU6050_SDA											GPIO_PC7_I2C0SDA
#define     I2CSlaveAddr										0x3B

/**********************************************************MPU6050寄存器地址************************************************/

#define SMPLRT_DIV 			0x19  				//陀螺仪采样率,典型值0x7(125HZ)
#define CONFIG 					0x1A 					//低通滤波频率,典型值0x06（5HZ）
#define GYRO_CONFIG 		0x1B 					//陀螺仪自检及测量范围,典型值:0x18(不自检 2000deg/s)
#define ACCEL_CONFIG 		0x1C 					//加速度自检,测量范围及高通滤波频率,典型值:0x01（不自检,2G,5HZ）
#define ACCEL_XOUT_H 		0x3B
#define ACCEL_XOUT_L 		0x3C
#define ACCEL_YOUT_H 		0x3D
#define ACCEL_YOUT_L 		0x3E
#define ACCEL_ZOUT_H 		0x3F
#define ACCEL_ZOUT_L 		0x40
#define TEMP_OUT_H 			0x41
#define TEMP_OUT_L 			0x42
#define GYRO_XOUT_H 		0x43
#define GYRO_XOUT_L 		0x44
#define GYRO_YOUT_H 		0x45
#define GYRO_YOUT_L 		0x46
#define GYRO_ZOUT_H 		0x47
#define GYRO_ZOUT_L 		0x48
#define USER_CTRL       0x6a
#define IICMasterControl 0x24
#define PWR_MGMT_1 			0x6B 					//电源管理,典型值:0x00(正常使用)
#define WHO_AM_I 				0x75          //I2C地址寄存器(默认数值0x68,只读)
#define SlaveAddress 		0xd0          //I2C写入时的地址字节数据,+1为读取
#define LSB_g 				 16384 					//LSB/g   即 16384=g=9.8m/(s*s)
#define deg_LSB 			10/164 					//deg/LSB
#define LSB_t 					 340 					//LSB/.C(温度)
typedef struct{
				short X;
				short Y;
				short Z;}S_INT16_XYZ;
typedef struct{
				float X;
				float Y;
				float Z;
                float X_OFFSET;
	            float Y_OFFSET;
                float Z_OFFSET;
	            short gyro[3];
	            long quat[4];
	            short accl[3];
	            short sensors;
	            unsigned char more;
                }_Q_ANGLE;
extern _Q_ANGLE  Q_ANGLE;
extern _Q_ANGLE  Q_AngleVelocity;
extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//最新一次读取值
extern int							GYRO_OFFSET_OK;
extern int							ACC_OFFSET_OK;
/**********************************************************MPU6050函数声明************************************************/
void I2C_Init(void);//模拟I2C初始化
void I2C_Start(void);//模拟I2C起始信号
void I2C_Stop(void);//模拟I2C停止信号
int I2C_Wait_Ack(void);//等待应答信号(返回值:false.接收应答失败,true：接收应答成功)
void I2C_Ack(void);//产生ACK应答
void I2C_NAck(void);//不产生Ack应答信号
void I2C_SendByte(unsigned char byte);//I2C发送一个字节
unsigned char I2C_ReadByte(unsigned char ack);//I2C读取一个字节Ack=1时,发送ACK，ACK=0时，发送NACK

void MPU6050_Init(void);
void I2C_WR_Reg(unsigned char regAddr,unsigned char val);//I2C写入寄存器一个字节
unsigned char I2C_RD_Reg(unsigned char regAddr);//I2C读取寄存器一个字节
void MPU6050_RD_AvgVal(short *x,short *y,short *z);//读取MPU6050坐标值10次后取平均值
short MPU6050_GetData(unsigned char regAddr);//将八位合成十六位数据
void MPU6050_RD_XYZ(short *x,short *y,short *z);//读取x,y,z轴加速度
void MPU6050_RD_AngleVelocity(short *ax,short *ay,short *az);//读取X,Y,Z轴角速度
void MPU6050_Read_Average(short *x,short *y,short *z,unsigned char times);//读取MPU6050数据times次,再取平均值
double MPU6050_Get_Angle(float x,float y,float z,unsigned char dir);//x,y,z为该方向重力加速度分量//dir为得到的角度
void delayms(unsigned int xms);
void delayus(unsigned int xus);
void MPU6050_Dataanl(void);
unsigned char I2C_ReadByte2(void);
unsigned long IIC_IO_MultipleWrite(unsigned char SAddress, unsigned char REG_Address, unsigned char *buf, int num);
unsigned char IIC_IO_MultipleRead(unsigned char  SAddress, unsigned char  REG_Address, unsigned char  *buf, int num);
#endif

