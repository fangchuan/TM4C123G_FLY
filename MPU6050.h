#ifndef _MPU6050_H
#define _MPU6050_H

/**********************************************************MPU6050���Ŷ���**************************************************/
#define  		SCL				GPIO_PIN_6			//PC6=SCL
#define     SDA				GPIO_PIN_7			//PC7=SDA

/**********************************************************MPU6050�⺯���궨��**********************************************/
#define  		SYSCTL_PERIPH_MPU6050_PORT     	SYSCTL_PERIPH_GPIOC
#define  		SYSCTL_PERIPH_MPU6050_I2C     	SYSCTL_PERIPH_I2C0
#define 	  MPU6050_PORT_BASE								GPIO_PORTC_BASE
#define 		MPU6050_I2C_BASE								I2C0_BASE
#define 		MPU6050_SCL											GPIO_PC6_I2C0SCL
#define 		MPU6050_SDA											GPIO_PC7_I2C0SDA
#define     I2CSlaveAddr										0x3B

/**********************************************************MPU6050�Ĵ�����ַ************************************************/

#define SMPLRT_DIV 			0x19  				//�����ǲ�����,����ֵ0x7(125HZ)
#define CONFIG 					0x1A 					//��ͨ�˲�Ƶ��,����ֵ0x06��5HZ��
#define GYRO_CONFIG 		0x1B 					//�������Լ켰������Χ,����ֵ:0x18(���Լ� 2000deg/s)
#define ACCEL_CONFIG 		0x1C 					//���ٶ��Լ�,������Χ����ͨ�˲�Ƶ��,����ֵ:0x01�����Լ�,2G,5HZ��
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
#define PWR_MGMT_1 			0x6B 					//��Դ����,����ֵ:0x00(����ʹ��)
#define WHO_AM_I 				0x75          //I2C��ַ�Ĵ���(Ĭ����ֵ0x68,ֻ��)
#define SlaveAddress 		0xd0          //I2Cд��ʱ�ĵ�ַ�ֽ�����,+1Ϊ��ȡ
#define LSB_g 				 16384 					//LSB/g   �� 16384=g=9.8m/(s*s)
#define deg_LSB 			10/164 					//deg/LSB
#define LSB_t 					 340 					//LSB/.C(�¶�)
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
extern S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;		//����һ�ζ�ȡֵ
extern int							GYRO_OFFSET_OK;
extern int							ACC_OFFSET_OK;
/**********************************************************MPU6050��������************************************************/
void I2C_Init(void);//ģ��I2C��ʼ��
void I2C_Start(void);//ģ��I2C��ʼ�ź�
void I2C_Stop(void);//ģ��I2Cֹͣ�ź�
int I2C_Wait_Ack(void);//�ȴ�Ӧ���ź�(����ֵ:false.����Ӧ��ʧ��,true������Ӧ��ɹ�)
void I2C_Ack(void);//����ACKӦ��
void I2C_NAck(void);//������AckӦ���ź�
void I2C_SendByte(unsigned char byte);//I2C����һ���ֽ�
unsigned char I2C_ReadByte(unsigned char ack);//I2C��ȡһ���ֽ�Ack=1ʱ,����ACK��ACK=0ʱ������NACK

void MPU6050_Init(void);
void I2C_WR_Reg(unsigned char regAddr,unsigned char val);//I2Cд��Ĵ���һ���ֽ�
unsigned char I2C_RD_Reg(unsigned char regAddr);//I2C��ȡ�Ĵ���һ���ֽ�
void MPU6050_RD_AvgVal(short *x,short *y,short *z);//��ȡMPU6050����ֵ10�κ�ȡƽ��ֵ
short MPU6050_GetData(unsigned char regAddr);//����λ�ϳ�ʮ��λ����
void MPU6050_RD_XYZ(short *x,short *y,short *z);//��ȡx,y,z����ٶ�
void MPU6050_RD_AngleVelocity(short *ax,short *ay,short *az);//��ȡX,Y,Z����ٶ�
void MPU6050_Read_Average(short *x,short *y,short *z,unsigned char times);//��ȡMPU6050����times��,��ȡƽ��ֵ
double MPU6050_Get_Angle(float x,float y,float z,unsigned char dir);//x,y,zΪ�÷����������ٶȷ���//dirΪ�õ��ĽǶ�
void delayms(unsigned int xms);
void delayus(unsigned int xus);
void MPU6050_Dataanl(void);
unsigned char I2C_ReadByte2(void);
unsigned long IIC_IO_MultipleWrite(unsigned char SAddress, unsigned char REG_Address, unsigned char *buf, int num);
unsigned char IIC_IO_MultipleRead(unsigned char  SAddress, unsigned char  REG_Address, unsigned char  *buf, int num);
#endif

