#include "cmd.h"
#include "driverlib/fpu.h"
#include "utils/uartstdio.h"
#include "UART_user.h"
#include "MPU6050.h"
#include "debug.h"
#include "AP_OpticalFlow_ADNS3080.h"

#define RECEIVE_BUFFER_SIZE 32
int32_t link_cmd_count=0;
int auto_land=0;
extern int sign;
extern int count_trans;
static enum		 //����ָ����յ�״̬
{
    FINDING_0x46 = 1,  //'F'��ASIIֵΪ0x46
    NEED_0x41,		   //��A����ASIIֵΪ0x41
    NEED_LENGTH,	   //'a'-'z','a'��ʾһλ����
    GETTING_DATA
} cmd_receiveStatus = FINDING_0x46;

static uint8_t  cmd_receiveLength = 0;//��Ч���ݵĳ��ȣ���������һλУ��λ
uint8_t  cmd_receiveBuffer[RECEIVE_BUFFER_SIZE];//��Ч���ݻ�����

//�Դ��ڽ������ݻ������ļ��
void cmd_recevie_checkEvent(void)
{
	uint8_t byte,i;
	uint8_t length = 0;
	switch(cmd_receiveStatus)
    {	
        case FINDING_0x46 :	//����Ƿ�������ͷ0X55
        {
						if(uart_receiveValidBufferSize() < 1)	//������ݻ��������ݳ���С��1���˳�
						break;
            uart_readReceiveBuffer(&byte,1);	//��ȡ���ݻ���������
            if(byte ==0x46) 	 //�����������ͷΪ0x46��������״̬ΪNEED_0x41
                cmd_receiveStatus = NEED_0x41;
        }
        break;
        case NEED_0x41 :
        {       
						if(uart_receiveValidBufferSize() < 1)
                break;
            uart_readReceiveBuffer(&byte,1);
            //
            if(byte == (uint8_t)(0x41))	//���������0x41��������״̬ΪNEED_LENGTH
                cmd_receiveStatus = NEED_LENGTH;
            else if(byte == (uint8_t)(0x46))
                break; // ��������0x46�������
            else
                cmd_receiveStatus = FINDING_0x46; //����ڶ�������ͷ����ȷ�������ñ�־ΪѰ�ҵ�һ��ͷ
        }
        break;
        case NEED_LENGTH :
        {            
			if(uart_receiveValidBufferSize() < 1)
                break; 
            uart_readReceiveBuffer(&length,1);
            length-=0x60;
			//
            if(length < 3 || length > RECEIVE_BUFFER_SIZE)//the length of data is in the range of(3,32)---xxH
                break;				  
            //
            cmd_receiveLength = length;
            cmd_receiveStatus = GETTING_DATA;
        }
        break;
        case GETTING_DATA :
        {
            if(uart_receiveValidBufferSize() < cmd_receiveLength)
                break;
            //
            uart_readReceiveBuffer(cmd_receiveBuffer,cmd_receiveLength);	//��ȡ����
            for(i=0;i<cmd_receiveLength;i++)
							cmd_receiveDataAnal(cmd_receiveBuffer,cmd_receiveLength);
            cmd_receiveStatus = FINDING_0x46;
        }
        break;
    }
}

void cmd_receiveDataAnal(uint8_t *buffer,uint8_t length)//����ȡ������Ϣ��FAX��������ݣ����д���
{

	switch(buffer[0])
	{
		case PROTOCOL_THROTTLE_SET:			//��������
			if(length!=3)
				break;
			control_up_or_down(buffer[1]);			
			break;
		case PROTOCOL_ATTITUDE_SET:			//��̬�趨
			if(length!=3)
				break;
			control_attitude_set(buffer[1]);
			break;
// 		case PROTOCOL_PID_CHANGE:		   //�ı�PID����
// 			if(length!=15)
// 				break;
			//pid_param_change(&buffer[1],length-2);
			break;
		case PROTOCOL_MODE_SET:			   //ģʽ����
			if(length!=3)
				break;
			control_on_or_off(buffer[1]);
			break;
		case PROTOCOL_LINK_OK:

			link_cmd_count++;
			break;
		case PROTOCOL_SET_H:     //0X7x,��������ݴ���10xcm
			if(length!=5)
				break;
			height_target=(buffer[1]-0X30)*10;
			break;
		case PROTOCOL_SET_DISTANCE:

			//control_set_distance(&buffer[1],length-2);
			break;	
	}
	buffer[0]=0x21;	//����һ��������������ֵ����֣������ظ�����
}
void control_attitude_set(char buffer)
{
	  if(begin_fly==1)
		{
			switch (buffer)
			{case PROTOCOL_ATTITUDE_FRONT://ǰ
				target_roll_degree -= 500;
			break;
			case PROTOCOL_ATTITUDE_BACK://��
				target_roll_degree += 500;
			break;
			case PROTOCOL_ATTITUDE_LEFT://��
				target_pitch_degree += 500;
			break;
			case PROTOCOL_ATTITUDE_RIGHT://��
				target_pitch_degree -= 500;
			break;

			case PROTOCOL_HOVER_HOLD://��ͣ
				alt_hold = 1;
			  hover_hold = 1;
			  height_base = 790;
			  rc_3.servo_out = height_base;
			break;
			case PROTOCOL_HEIGHT_HOLD:
				alt_hold=1;
			  height_base = 790;
			  rc_3.servo_out = height_base;
			break;
		  }
    }
}
void control_on_or_off(char buffer)
{
	  switch(buffer)
	 {
		 case PROTOCOL_MODE_HALT:begin_fly=0;PWM_Set(1230,1230,1230,1230);break;
		 case PROTOCOL_MODE_START:begin_fly=1;break;
   }
}
void control_up_or_down(char buffer)
{
	  if(begin_fly==1)
	{
	  switch(buffer)
	 {
		 case PROTOCOL_THROTTLE_UP:rc_3.servo_out += 50;break;
		 case PROTOCOL_THROTTLE_DOWN:rc_3.servo_out -= 20;break;
   }
  }
}
extern int alt_hold;
extern int hover_hold;
void Cmd_ReceiveHandler(char *buffer)//
{
  if(sign==1)
	{
		if(buffer[0]==0x46)
	{
		if(buffer[1]==0x41)
	{
		if(buffer[7]==0x48)
	{
	switch(buffer[3])
	{
		case PROTOCOL_THROTTLE_SET:			//��������
			alt_hold = 0;
			control_up_or_down(buffer[4]);			
			break;
		case PROTOCOL_ATTITUDE_SET:			//��̬�趨
			hover_hold = 0;
			control_attitude_set(buffer[4]);
			break;
		case PROTOCOL_OP_RESET:		   //�Թ����趨
			if(buffer[4]==0x30)Reset_ADNS3080();
		  if(buffer[4]==0x31){target_point_x = current_point_x;target_point_y = current_point_y;}
			break;
		case PROTOCOL_MODE_SET:			   //ģʽ����
			control_on_or_off(buffer[4]);
			break;
		case PROTOCOL_TARGET_ATTITUDE_LOCK:
			//control_target_attitude();
		target_roll_degree=(buffer[4]-0x30)*100;
		target_pitch_degree=(buffer[5]-0x30)*100;
		target_yaw_degree=(buffer[6]-0x30)*100;
		case PROTOCOL_LINK_OK:

			link_cmd_count++;
			break;
		case PROTOCOL_SET_H:     //0X7x,��������ݴ���100x mm
			alt_hold=1;
		  height_target=(buffer[4]-0X30)*100+(buffer[5]-0x30)*10+ (buffer[6]-0x30);
		  
			break;
		case PROTOCOL_SET_DISTANCE:
			//control_set_distance(&buffer[4],length-2);
		hover_hold = 1;
		if(buffer[2] == 'e')
		    target_point_y = ((buffer[4]-0x30)*100+(buffer[5]-0x30)*10+(buffer[6]-0x30))*0.1;
		if(buffer[2] == 'd')
			  target_point_x = ((buffer[4]-0x30)*100+(buffer[5]-0x30)*10+(buffer[6]-0x30))*0.1;
			break;	
		case PROTOCOL_AUTO_LANDING:
			alt_hold=0;
		  auto_land = 1;
		  break;
	}
	buffer[3]=0x21;	//����һ��������������ֵ����֣������ظ�����
  }
  }
  }
  }
}

//to deal with the case of lost in
//1s  called
void exception(void)
{
	 static int i=0;
	  i++;
	 if(i==15)
	 {
		i=0;
	  if(link_cmd_count <= 1 && begin_fly == 1) 
		{
			//begin_fly=0;
			auto_land = 1;
			//PWM_Set(1230,1230,1230,1230);
		}		
		
		link_cmd_count=0;
	 }
	 if(fabs(ap_ahrs_dcm.roll_sensor) >=4500 ||
	  fabs(ap_ahrs_dcm.pitch_sensor) >= 4500)
	 {
		 begin_fly = 0;
		 PWM_Set(1230,1230,1230,1230);
   }
	 if(auto_land ==1) auto_landing();
}
//Autonomous landing
//
void auto_landing(void)
{
	  alt_hold =0;
	  rc_3.servo_out -= 8;
}

void return_angle(long roll,long pitch,long yaw)
{

	    debug_printf("BE:%ld$%ld$%ld$\n",roll,pitch,yaw);
}
void return_hight(long h)
{
	     debug_printf("BH:%ld$\n",h);
}

void return_motor(AP_MotorsQuard*ap_motors_quard)
{
	  debug_printf("BM:%ld$%ld$%ld$%ld\n", ap_motors_quard->motor_out[0], ap_motors_quard->motor_out[1],
  	ap_motors_quard->motor_out[2], ap_motors_quard->motor_out[3]);
	 
}
void return_key()
{
	  debug_printf("begin_fly:%d\n!",begin_fly);
}
void return_position()
{
	  //debug_printf("BX:%d\n",current_point_x);
		debug_printf("BP:%d$%d\n",current_point_x,current_point_y);
}
void return_target_angle()
{
	  //debug_printf("BR:%d\n",target_roll_degree);
	  debug_printf("BT:%d$%d\n",target_roll_degree,target_pitch_degree);
}