/*
 * ccdandcsb.h
 *
 *  Created on: 2014-8-9
 *      Author: Administrator
 */

#ifndef CCDANDCSB_H_
#define CCDANDCSB_H_



void intCHAOSHENGBO(void);					//��������ʼ��
int CHAOSHENGBO(void);						//����������			���ؾ���cm
void InitCSB_IO(void);					
void GPIO_INT_E(void);
void CSB_FS(void);

int adc1(void);							//����ͷad�ɼ�
void intadc(void);							//����ͷad��ʼ��
void intccd(void);							//����ͷ��ʼ��
void caiji(void);							//����ͷ�ɼ�
int ccd(void);							    //����ͷ�ɼ�������    ����	{100 ����ת		10��ֱ��		1����ת	0��û�����}






#endif /* CCDANDCSB_H_ */
