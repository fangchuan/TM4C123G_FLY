#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "TM4C.h"
#include "MPU6050.h"
#include "UART_user.h"
S_INT16_XYZ		MPU6050_ACC_LAST,MPU6050_GYRO_LAST;	
int							GYRO_OFFSET_OK;
int							ACC_OFFSET_OK;
S_INT16_XYZ		GYRO_OFFSET,ACC_OFFSET;			//��Ư
_Q_ANGLE  Q_ANGLE;
_Q_ANGLE  Q_AngleVelocity;
void MPU6050_Init(void)
{
	I2C_Init();//ģ��I2C��ʼ��
	I2C_WR_Reg(PWR_MGMT_1,0x00);//�������״̬
	I2C_WR_Reg(SMPLRT_DIV,0x07);//���������ǲ�����
	I2C_WR_Reg(CONFIG,0x03);//���õ�ͨ�˲�Ƶ��44HZ,and gryo out put rate=1KHZ
	I2C_WR_Reg(GYRO_CONFIG,0x00);//�������Լ켰������Χ,����ֵ:0x18(���Լ� 250deg/s)
	I2C_WR_Reg(ACCEL_CONFIG,0x08);//���ٶ��Լ�,������Χ����ͨ�˲�Ƶ��,����ֵ:0x01�����Լ�,4G,5HZ��
	//I2C_WR_Reg(IICMasterControl,0x13);
	
		
}

void I2C_Init(void)//ģ��I2C��ʼ��
{
	SysCtlPeripheralEnable(SYSCTL_PERIPH_MPU6050_PORT);//ʹ������ʱ��GPIOC
	GPIOPinTypeGPIOOutputOD(MPU6050_PORT_BASE,SDA|SCL);//SCL&SDA�����
	GPIOPadConfigSet(MPU6050_PORT_BASE, SCL|SDA, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);//GPIO��������Ϊ������ģʽ
	GPIOPinWrite(MPU6050_PORT_BASE,SCL|SDA,SCL|SDA);//����GPIO����Ϊ�ߵ�ƽ

}

void I2C_Start(void)//ģ��I2C��ʼ�ź�
{
	GPIOPinTypeGPIOOutputOD(MPU6050_PORT_BASE,SDA);//SDA�����
	GPIOPinWrite(MPU6050_PORT_BASE,SCL|SDA,SCL|SDA);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SDA,0x00);//����GPIO����Ϊ�͵�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//ǯסI2C����,׼�����ͻ��������
}

void I2C_Stop(void)//ģ��I2Cֹͣ�ź�
{
	GPIOPinTypeGPIOOutputOD(MPU6050_PORT_BASE,SDA);//SDA�����
	GPIOPinWrite(MPU6050_PORT_BASE,SCL|SDA,0x00);//����GPIO����Ϊ�͵�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL|SDA,SCL|SDA);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);

}
int I2C_Wait_Ack(void)//�ȴ�Ӧ���ź�(����ֵ:false.����Ӧ��ʧ��,true������Ӧ��ɹ�)
{
	uint8_t  ucErrTime=0;
	GPIODirModeSet(MPU6050_PORT_BASE,SDA,GPIO_DIR_MODE_IN);//SDA����Ϊ����
	GPIOPadConfigSet(MPU6050_PORT_BASE, SCL|SDA, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);//GPIO��������Ϊ������ģʽ
	GPIOPinWrite(MPU6050_PORT_BASE,SDA,SDA);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,SCL);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);
	while(GPIOPinRead(MPU6050_PORT_BASE,SDA)==SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			I2C_Stop();
			return 0;
		}
	}
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ
		return 1;
}
void I2C_Ack(void)//����ACKӦ��
{
	GPIOPinTypeGPIOOutputOD(MPU6050_PORT_BASE,SDA);//SDA�����
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SDA,0x00);//����GPIO����Ϊ�͵�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,SCL);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ
}
void I2C_NAck(void)//������AckӦ���ź�
{
	GPIOPinTypeGPIOOutputOD(MPU6050_PORT_BASE,SDA);//SDA�����
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ
	GPIOPinWrite(MPU6050_PORT_BASE,SDA,SDA);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,SCL);//����GPIO����Ϊ�ߵ�ƽ
	delayus(5);
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ
	

}
void I2C_SendByte(unsigned char byte)//I2C����һ���ֽ�
{
	uint8_t i=0;
	GPIOPinTypeGPIOOutputOD(MPU6050_PORT_BASE,SDA);//SDA�����
	GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ(��ʼ���ݴ���)
	for(i=0;i<8;i++)
	{
			if((byte &0x80)==0x00)
			{
				GPIOPinWrite(MPU6050_PORT_BASE,SDA,0x00);//����GPIO����Ϊ�͵�ƽ
			}
			else
			{
				GPIOPinWrite(MPU6050_PORT_BASE,SDA,SDA);//����GPIO����Ϊ�ߵ�ƽ	
			}
			delayus(5);
			GPIOPinWrite(MPU6050_PORT_BASE,SCL,SCL);//����GPIO����Ϊ�ߵ�ƽ
			delayus(5);
			GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ
			delayus(5);
			byte<<=1;
	}

}
unsigned char I2C_ReadByte(unsigned char ack)//I2C��ȡһ���ֽ�Ack=1ʱ,����ACK��ACK=0ʱ������NACK
{
	unsigned char i=0,byte=0;
	GPIODirModeSet(MPU6050_PORT_BASE,SDA,GPIO_DIR_MODE_IN);//SDA����Ϊ����
	GPIOPadConfigSet(MPU6050_PORT_BASE, SCL|SDA, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);//GPIO��������Ϊ������ģʽ
	for(i=0;i<8;i++)
	{
		GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ(��ʼ���ݴ���)
		delayus(5);
		byte<<=1;
		if(GPIOPinRead(MPU6050_PORT_BASE,SDA)==SDA)
		byte |=0x01;
		GPIOPinWrite(MPU6050_PORT_BASE,SCL,SCL);//����GPIO����Ϊ�ߵ�ƽ
		delayus(5);
	}
	if(ack==0)
		I2C_NAck();//����nACK
	else
		I2C_Ack();//����ACK
	return byte;

}
unsigned char I2C_ReadByte2()
{
	unsigned char i=0,byte=0;
	GPIODirModeSet(MPU6050_PORT_BASE,SDA,GPIO_DIR_MODE_IN);//SDA����Ϊ����
	GPIOPadConfigSet(MPU6050_PORT_BASE, SCL|SDA, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);//GPIO��������Ϊ������ģʽ
	for(i=0;i<8;i++)
	{
		GPIOPinWrite(MPU6050_PORT_BASE,SCL,0x00);//����GPIO����Ϊ�͵�ƽ(��ʼ���ݴ���)
		delayus(5);
		byte<<=1;
		if(GPIOPinRead(MPU6050_PORT_BASE,SDA)==SDA)
		byte |=0x01;
		GPIOPinWrite(MPU6050_PORT_BASE,SCL,SCL);//����GPIO����Ϊ�ߵ�ƽ
		delayus(5);
	}
	return byte;

}
void I2C_WR_Reg(unsigned char regAddr,unsigned char value)//I2Cд��Ĵ���һ���ֽ�
{
			I2C_Start();//ģ��I2C��ʼ�ź�
			I2C_SendByte(SlaveAddress);//�����豸��ַ+д�ź�
			I2C_Wait_Ack();//����Ӧ���ź�
			I2C_SendByte(regAddr);//�����ڲ��Ĵ�����ַ
			I2C_Wait_Ack();//����Ӧ���ź�
			I2C_SendByte(value);//����д���ڲ��Ĵ�������
			I2C_Wait_Ack();//����Ӧ���ź�
			I2C_Stop();//����ֹͣ�ź�

}

unsigned char I2C_RD_Reg(unsigned char regAddr)//I2C��ȡ�Ĵ���һ���ֽ�
{
		unsigned char value=0;
		I2C_Start();//ģ��I2C��ʼ�ź�
		I2C_SendByte(SlaveAddress);//�����豸��ַ+д�ź�
		I2C_Wait_Ack();//����Ӧ���ź�
		I2C_SendByte(regAddr);//�����ڲ��Ĵ�����ַ
	  I2C_Wait_Ack();//����Ӧ���ź�
		I2C_Start();//ģ��I2C��ʼ�ź�
		I2C_SendByte(SlaveAddress+1);//�����豸��ַ+���ź�
		I2C_Wait_Ack();//����Ӧ���ź�
		value=I2C_ReadByte(0);//��ȡ�Ĵ���������(������Ӧ���ź�)
		I2C_Stop();//����ֹͣ�ź�
		return value;
}
unsigned long IIC_IO_MultipleWrite(unsigned char SAddress, unsigned char REG_Address, unsigned char *buf, int num)
{
    long ret = 0;
    int i;
		I2C_Start();//ģ��I2C��ʼ�ź�
		I2C_SendByte(SAddress<<1);//�����豸��ַ+д�ź�
		ret |=I2C_Wait_Ack();//����Ӧ���ź�
	  //I2C_Wait_Ack();
		I2C_SendByte(REG_Address);//�����ڲ��Ĵ�����ַ
		ret |=I2C_Wait_Ack();//����Ӧ���ź�
	  //I2C_Wait_Ack();

    for(i = 0; i < num; i++)
    {
					I2C_SendByte(buf[i]);//����д���ڲ��Ĵ�������
			    ret |=I2C_Wait_Ack();//����Ӧ���ź�
			   //I2C_Wait_Ack();
     
    }
    I2C_Stop();    
    
    return ret^1;//iic success return 0,or fei0
		
}
//?????*****************************************
unsigned char IIC_IO_MultipleRead(unsigned char  SAddress, unsigned char  REG_Address, unsigned char  *buf, int num)
{   
    unsigned char ret = 0;
    int i;
		I2C_Start();//ģ��I2C��ʼ�ź�
		I2C_SendByte(SAddress<<1);//�����豸��ַ+д�ź�
		ret |=I2C_Wait_Ack();//����Ӧ���ź�
		I2C_SendByte(REG_Address);//�����ڲ��Ĵ�����ַ
		ret |=I2C_Wait_Ack();//����Ӧ���ź�
    if(ret^1)
    {
        I2C_Stop();
        return ret^1;
    }
    
    I2C_Start();                          //????
    I2C_SendByte(SAddress <<1 |0x01);  //??????+???
		ret |= I2C_Wait_Ack();
    if(ret^1)
    {
        I2C_Stop();
        return ret^1;
    }
    for (i=0; i< num; i++)                      //????6?????,???BUF
    {

        if (i == num-1)
        {
					buf[i] = I2C_ReadByte(0);

        }
        else
        {
					buf[i] = I2C_ReadByte(1);

       }
   }
    I2C_Stop();  
  
    return ret^1;//iic success return 0,or fei0
	
}

void MPU6050_RD_AvgVal(short *x,short *y,short *z)//��ȡMPU6050����ֵ10�κ�ȡƽ��ֵ
{
	 unsigned int tx=0,ty=0,tz=0;
	 unsigned char i=0;
	 for(i=0;i<10;i++)
	{
			MPU6050_RD_XYZ(x,y,z);
			delayms(10);
			tx=tx+*x;		ty=ty+*y;		tz=tz+*z;

	}
	*x=tx/10;  	*y=ty/10;		*z=tz/10;
}

short MPU6050_GetData(unsigned char regAddr)//����λ�ϳ�ʮ��λ����
{
	short  value=0;
	value =I2C_RD_Reg(regAddr)<<8;
	value |=I2C_RD_Reg(regAddr+1);

	return value;

}

void MPU6050_RD_XYZ(short *x,short *y,short *z)//��ȡx,y,z������ֵ
{
	*x=MPU6050_GetData(ACCEL_XOUT_H);
	*y=MPU6050_GetData(ACCEL_YOUT_H);
	*z=MPU6050_GetData(ACCEL_ZOUT_H);

}

void MPU6050_RD_AngleVelocity(short *ax,short *ay,short *az)//��ȡX,Y,Z����ٶ�
{
	*ax=MPU6050_GetData(GYRO_XOUT_H);
	*ay=MPU6050_GetData(GYRO_YOUT_H);
	*az=MPU6050_GetData(GYRO_ZOUT_H);	
	
}

void MPU6050_Read_Average(short *x,short *y,short *z,unsigned char times)//��ȡMPU6050����times��,��ȡƽ��ֵ
{
	unsigned char i=0;
	short tx=0,ty=0,tz=0;
	*x=0;		*y=0;		*z=0;
	if(times>0)//��ȡ����>0
	{
			for(i=0;i<times;i++)
			{
					MPU6050_RD_XYZ(&tx,&ty,&tz);
					*x=*x+tx;		*y=*y+ty;		*z=*z+tz;
					delayms(5);
			}
			*x=*x/times;		*y=*y/times;    *z=*z/times;
	}
	
}

double MPU6050_Get_Angle(float x,float y,float z,unsigned char dir)//x,y,zΪ�÷����������ٶȷ���//dirΪ�õ��ĽǶ�
{
	double temp=0,res=0;
	switch(dir)
	{
		case 0: //����ȻZ��ĽǶ�
			temp=sqrt(x*x+y*y)/z;
			res=atan(temp);
			break;
		case 1: //����ȻX��ĽǶ�
			temp=x/sqrt(y*y+z*z);
			res=atan(temp);
			break;
		case 2: //����ȻY��ĽǶ�
			temp=y/sqrt(x*x+z*z);
			res=atan(temp);
			break;

	}
	temp=(res*1800/3.14);//�����ƻ���ɽǶ���
	return temp;

}
void delayms(unsigned int xms)
{
	SysCtlDelay(xms*(SysCtlClockGet()/3000));
}
void delayus(unsigned int xus)
{
	SysCtlDelay(xus*(SysCtlClockGet()/3000000));
}
void MPU6050_Dataanl()
{
	MPU6050_RD_XYZ(&MPU6050_ACC_LAST.X,&MPU6050_ACC_LAST.Y,&MPU6050_ACC_LAST.Z);
	MPU6050_RD_AngleVelocity(&MPU6050_GYRO_LAST.X,&MPU6050_GYRO_LAST.Y,&MPU6050_GYRO_LAST.Z);
//   MPU6050_ACC_LAST.X-= ACC_OFFSET.X;
// 	MPU6050_ACC_LAST.Y-= ACC_OFFSET.Y;
// 	MPU6050_ACC_LAST.Z-=ACC_OFFSET.Z;
// 	//�����¶�ADC
// 	MPU6050_GYRO_LAST.X-=GYRO_OFFSET.X;
// 	MPU6050_GYRO_LAST.Y-=GYRO_OFFSET.Y;
// 	MPU6050_GYRO_LAST.Z-=GYRO_OFFSET.Z;
// 	
// 	if(!GYRO_OFFSET_OK)
// 	{
// 		static long	tempgx=0,tempgy=0,tempgz=0;
// 		static int cnt_g=0;
// // 		LED1_ON;
// 		if(cnt_g==0)
// 		{
// 			GYRO_OFFSET.X=0;
// 			GYRO_OFFSET.Y=0;
// 			GYRO_OFFSET.Z=0;
// 			tempgx = 0;
// 			tempgy = 0;
// 			tempgz = 0;
// 			cnt_g = 1;
// 			return;
// 		}
// 		tempgx+= MPU6050_GYRO_LAST.X;
// 		tempgy+= MPU6050_GYRO_LAST.Y;
// 		tempgz+= MPU6050_GYRO_LAST.Z;
// 		if(cnt_g==200)
// 		{
// 			GYRO_OFFSET.X=tempgx/cnt_g;
// 			GYRO_OFFSET.Y=tempgy/cnt_g;
// 			GYRO_OFFSET.Z=tempgz/cnt_g;
// 			cnt_g = 0;
// 			GYRO_OFFSET_OK = 1;
// //			EE_SAVE_GYRO_OFFSET();//��������
// 			return;
// 		}
// 		cnt_g++;
// 	}
// 	if(!ACC_OFFSET_OK)
// 	{
// 		static int32_t	tempax=0,tempay=0,tempaz=0;
// 		static uint8_t cnt_a=0;
// // 		LED1_ON;
// 		if(cnt_a==0)
// 		{
// 			ACC_OFFSET.X = 0;
// 			ACC_OFFSET.Y = 0;
// 			ACC_OFFSET.Z = 0;
// 			tempax = 0;
// 			tempay = 0;
// 			tempaz = 0;
// 			cnt_a = 1;
// 			return;
// 		}
// 		tempax+= MPU6050_ACC_LAST.X;
// 		tempay+= MPU6050_ACC_LAST.Y;
// 		//tempaz+= MPU6050_ACC_LAST.Z;
// 		if(cnt_a==200)
// 		{
// 			ACC_OFFSET.X=tempax/cnt_a;
// 			ACC_OFFSET.Y=tempay/cnt_a;
// 			ACC_OFFSET.Z=tempaz/cnt_a;
// 			cnt_a = 0;
// 			ACC_OFFSET_OK = 1;
// //			EE_SAVE_ACC_OFFSET();//��������
// 			return;
// 		}
// 		cnt_a++;		
// 	}
}
