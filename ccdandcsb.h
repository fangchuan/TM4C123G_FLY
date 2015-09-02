/*
 * ccdandcsb.h
 *
 *  Created on: 2014-8-9
 *      Author: Administrator
 */

#ifndef CCDANDCSB_H_
#define CCDANDCSB_H_



void intCHAOSHENGBO(void);					//超声波初始化
int CHAOSHENGBO(void);						//超声波调用			返回距离cm
void InitCSB_IO(void);					
void GPIO_INT_E(void);
void CSB_FS(void);

int adc1(void);							//摄像头ad采集
void intadc(void);							//摄像头ad初始化
void intccd(void);							//摄像头初始化
void caiji(void);							//摄像头采集
int ccd(void);							    //摄像头采集及处理    返回	{100 ：右转		10：直行		1：左转	0：没看清楚}






#endif /* CCDANDCSB_H_ */
