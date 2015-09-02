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
static enum		 //定义指令接收的状态
{
    FINDING_0x46 = 1,  //'F'的ASII值为0x46
    NEED_0x41,		   //‘A’的ASII值为0x41
    NEED_LENGTH,	   //'a'-'z','a'表示一位数据
    GETTING_DATA
} cmd_receiveStatus = FINDING_0x46;

static uint8_t  cmd_receiveLength = 0;//有效数据的长度，包括最后的一位校验位
uint8_t  cmd_receiveBuffer[RECEIVE_BUFFER_SIZE];//有效数据缓存区

//对串口接收数据缓冲区的检测
void cmd_recevie_checkEvent(void)
{
	uint8_t byte,i;
	uint8_t length = 0;
	switch(cmd_receiveStatus)
    {	
        case FINDING_0x46 :	//检测是否有数据头0X55
        {
						if(uart_receiveValidBufferSize() < 1)	//如果数据缓冲区数据长度小于1，退出
						break;
            uart_readReceiveBuffer(&byte,1);	//读取数据缓冲区数据
            if(byte ==0x46) 	 //如果读到数据头为0x46，则设置状态为NEED_0x41
                cmd_receiveStatus = NEED_0x41;
        }
        break;
        case NEED_0x41 :
        {       
						if(uart_receiveValidBufferSize() < 1)
                break;
            uart_readReceiveBuffer(&byte,1);
            //
            if(byte == (uint8_t)(0x41))	//如果读到了0x41，则设置状态为NEED_LENGTH
                cmd_receiveStatus = NEED_LENGTH;
            else if(byte == (uint8_t)(0x46))
                break; // 连续两个0x46的情况。
            else
                cmd_receiveStatus = FINDING_0x46; //如果第二个数据头不正确，则重置标志为寻找第一个头
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
            uart_readReceiveBuffer(cmd_receiveBuffer,cmd_receiveLength);	//读取数据
            for(i=0;i<cmd_receiveLength;i++)
							cmd_receiveDataAnal(cmd_receiveBuffer,cmd_receiveLength);
            cmd_receiveStatus = FINDING_0x46;
        }
        break;
    }
}

void cmd_receiveDataAnal(uint8_t *buffer,uint8_t length)//将获取到的信息（FAX后面的数据）进行处理
{

	switch(buffer[0])
	{
		case PROTOCOL_THROTTLE_SET:			//油门设置
			if(length!=3)
				break;
			control_up_or_down(buffer[1]);			
			break;
		case PROTOCOL_ATTITUDE_SET:			//姿态设定
			if(length!=3)
				break;
			control_attitude_set(buffer[1]);
			break;
// 		case PROTOCOL_PID_CHANGE:		   //改变PID参数
// 			if(length!=15)
// 				break;
			//pid_param_change(&buffer[1],length-2);
			break;
		case PROTOCOL_MODE_SET:			   //模式设置
			if(length!=3)
				break;
			control_on_or_off(buffer[1]);
			break;
		case PROTOCOL_LINK_OK:

			link_cmd_count++;
			break;
		case PROTOCOL_SET_H:     //0X7x,后面的数据代表10xcm
			if(length!=5)
				break;
			height_target=(buffer[1]-0X30)*10;
			break;
		case PROTOCOL_SET_DISTANCE:

			//control_set_distance(&buffer[1],length-2);
			break;	
	}
	buffer[0]=0x21;	//任意一个不会在上面出现的数字，避免重复运行
}
void control_attitude_set(char buffer)
{
	  if(begin_fly==1)
		{
			switch (buffer)
			{case PROTOCOL_ATTITUDE_FRONT://前
				target_roll_degree -= 500;
			break;
			case PROTOCOL_ATTITUDE_BACK://后
				target_roll_degree += 500;
			break;
			case PROTOCOL_ATTITUDE_LEFT://左
				target_pitch_degree += 500;
			break;
			case PROTOCOL_ATTITUDE_RIGHT://右
				target_pitch_degree -= 500;
			break;

			case PROTOCOL_HOVER_HOLD://悬停
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
		case PROTOCOL_THROTTLE_SET:			//油门设置
			alt_hold = 0;
			control_up_or_down(buffer[4]);			
			break;
		case PROTOCOL_ATTITUDE_SET:			//姿态设定
			hover_hold = 0;
			control_attitude_set(buffer[4]);
			break;
		case PROTOCOL_OP_RESET:		   //对光流设定
			if(buffer[4]==0x30)Reset_ADNS3080();
		  if(buffer[4]==0x31){target_point_x = current_point_x;target_point_y = current_point_y;}
			break;
		case PROTOCOL_MODE_SET:			   //模式设置
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
		case PROTOCOL_SET_H:     //0X7x,后面的数据代表100x mm
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
	buffer[3]=0x21;	//任意一个不会在上面出现的数字，避免重复运行
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