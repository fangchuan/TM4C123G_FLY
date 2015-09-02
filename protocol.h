#ifndef __PROTOCOL_H_
#define __PROTOCOL_H_
#define PROTOCOL_VERSION 0 // 协议版本
//上位机端为字符串发送'FA+长度+类型+参数+H'
#define PROTOCOL_START_0                 0x46//数据头F
#define PROTOCOL_START_1                 0x41//A
#define PROTOCOL_END					 0x48//数据尾H


#define PROTOCOL_SERVO_SET				 0x30//舵机设置
#define PROTOCOL_SERVO_UP				 0x30
#define PROTOCOL_SERVO_DOWN				 0x31
#define PROTOCOL_SERVO_LEFT				 0x32
#define PROTOCOL_SERVO_RIGHT			 0x33


#define PROTOCOL_THROTTLE_SET			 0x31//基础油门设置
#define PROTOCOL_THROTTLE_UP			 0x30
#define PROTOCOL_THROTTLE_DOWN		     0x31
#define PROTOCOL_THROTTLE_LITTLE_UP		 0x32
#define PROTOCOL_THROTTLE_LITTLE_DOWN	 0x33

#define PROTOCOL_ATTITUDE_SET			 0x32//姿态控制
#define PROTOCOL_ATTITUDE_FRONT		     0x30
#define PROTOCOL_ATTITUDE_BACK			 0x31
#define PROTOCOL_ATTITUDE_LEFT			 0x32
#define PROTOCOL_ATTITUDE_RIGHT			 0x33
#define PROTOCOL_HOVER_HOLD			 0x34//悬停模式
#define PROTOCOL_HEIGHT_HOLD         0x35//定高模式

#define PROTOCOL_OP_RESET		 		 0x33//复位光流
#define	PROTOCOL_PID_X_CHANGE			 0x30
#define	PROTOCOL_PID_Y_CHANGE 			 0x31
#define	PROTOCOL_PID_Z_CHANGE			 0x32
#define PROTOCOL_PID_H_CHANGE				0x33
#define PROTOCOL_PID_A_CHANGE				0x34

#define PROTOCOL_MODE_SET				 0x34//模式选择
#define PROTOCOL_MODE_HALT				 0x30//停机模式
#define PROTOCOL_MODE_START	     0x31//开机
#define PROTOCOL_MODE_H_HOLD_FRONT			0x32
#define PROTOCOL_MODE_H_HOLD_HOVER			0x33

#define PROTOCOL_TARGET_ATTITUDE_LOCK	 0x35//从指令中获取目标姿态

#define PROTOCOL_LINK_OK			0x36//数据通信正常FAbLH

#define PROTOCOL_SET_H			0x37//设置定高时的高度

#define PROTOCOL_SET_DISTANCE 0x38

#define PROTOCOL_AUTO_LANDING 0x39
#endif
