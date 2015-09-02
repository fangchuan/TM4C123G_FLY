#include "TM4C.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usart.h"

enum
{
    UART_SUCCEED = 0,
    UART_FAILED  = 1
};	
static  struct 
{
    uint32_t head_transmit;//����ͷ
    uint32_t tail_transmit;//����β
    uint32_t head_receive;//����ͷ
    uint32_t tail_receive;//����β
    uint8_t buffer_transmit[UART_BUFFER_SIZE_TRANSMIT];//���ݷ��ͻ�����
    uint8_t buffer_receive[UART_BUFFER_SIZE_RECEIVE];//���ݽ��ջ�����
}uart;
// void return_high(void)
// {
// 		UARTprintf("BH\n%d\n!",high/10-8);
// }

// ��ȡ���ͻ��������пռ��С��
int32_t uart_transmitIdleBufferSize(void)
{
	if(uart.tail_transmit <= uart.head_transmit)
		return UART_BUFFER_SIZE_TRANSMIT - uart.head_transmit + uart.tail_transmit;
	return uart.tail_transmit - uart.head_transmit - 1; // -1:������������
}

// ��ȡ���ͻ������е����ݳ��ȡ�
int32_t uart_transmitValidBufferSize(void)
{
	if(uart.tail_transmit <= uart.head_transmit)
		return uart.head_transmit - uart.tail_transmit;
	return uart.head_transmit + (UART_BUFFER_SIZE_TRANSMIT - uart.tail_transmit);
}

// �ӷ��ͻ�����ȡ��һ���ֽڣ����ͳ�ȥ��
// ����黺�����Ƿ�գ�����鷢��״̬��
void uart_transmitAByte(void)
{
   //USART_SendData(USART3,uart.buffer_transmit[uart.tail_transmit]);
		uart.tail_transmit ++;
		if(uart.tail_transmit >= UART_BUFFER_SIZE_TRANSMIT)
		uart.tail_transmit = 0;
}

// ��һ�����ݷ��뷢�ͻ�������
// ����ֵ��{uart_SUCCEED,uart_FAILED}
uint8_t uart_transmit(const void * data,uint32_t len)
{
	int i;
	if(uart_transmitIdleBufferSize() < len)
	{
		return UART_FAILED; // �ռ䲻����
	}
	//
	for(i=0;i<len;i++)
	{
		uart.buffer_transmit[uart.head_transmit] = ((uint8_t *)data)[i];
		//
		uart.head_transmit ++;
		if(uart.head_transmit >= UART_BUFFER_SIZE_TRANSMIT)
			uart.head_transmit = 0;
		//
	}
	//
	// �������û�����ڽ��У����������͡�
	//if(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == SET)
        uart_transmitAByte();
	//
	return UART_SUCCEED;
}

// ��ȡ���ջ������е����ݳ��ȡ�
int32_t uart_receiveValidBufferSize(void)
{
	if(uart.tail_receive <= uart.head_receive)
		return uart.head_receive - uart.tail_receive;
	return uart.head_receive + (UART_BUFFER_SIZE_RECEIVE - uart.tail_receive);
}

// ��һ���ֽڷ�����ջ�������
// ����������ʧ�ܡ�
// ����ֵ��{uart_SUCCEED,uart_FAILED}��
uint8_t uart_receivePushToBuffer(uint8_t data)
{
    uint32_t newHead = uart.head_receive + 1;
	if(newHead == UART_BUFFER_SIZE_RECEIVE)
		newHead = 0;
    //
    if(newHead == uart.tail_receive)
        return UART_FAILED;
    //
    uart.buffer_receive[uart.head_receive] = data;
	uart.head_receive = newHead;
	//
    return UART_SUCCEED;
}

// �ӽ��ջ�����ȡ�����ݣ�����ȡ���ĳ��ȡ�
// ȡ���ĳ���ΪoutputBufferLength��uart_receiveValidBufferSize()����С�ߡ�
int32_t uart_readReceiveBuffer(void * outputBuffer,int32_t outputBufferLength)
{
    // ����outputBufferLength��uart_receiveValidBufferSize()����Сֵ��
    uint32_t returnLength = uart_receiveValidBufferSize();
	  uint32_t i;
    if(outputBufferLength < returnLength)
        returnLength = outputBufferLength;
    //
    // �������ݣ��ƽ�ָ�롣
    for(i=0;i<returnLength;i++)
    {
        ((uint8_t *)outputBuffer)[i] = uart.buffer_receive[uart.tail_receive];
        //
        uart.tail_receive ++;
        if(uart.tail_receive == UART_BUFFER_SIZE_RECEIVE)
            uart.tail_receive = 0;
    }
    //
    return returnLength;
}


