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
    uint32_t head_transmit;//发送头
    uint32_t tail_transmit;//发送尾
    uint32_t head_receive;//接收头
    uint32_t tail_receive;//接收尾
    uint8_t buffer_transmit[UART_BUFFER_SIZE_TRANSMIT];//数据发送缓冲区
    uint8_t buffer_receive[UART_BUFFER_SIZE_RECEIVE];//数据接收缓冲区
}uart;
// void return_high(void)
// {
// 		UARTprintf("BH\n%d\n!",high/10-8);
// }

// 获取发送缓冲区空闲空间大小。
int32_t uart_transmitIdleBufferSize(void)
{
	if(uart.tail_transmit <= uart.head_transmit)
		return UART_BUFFER_SIZE_TRANSMIT - uart.head_transmit + uart.tail_transmit;
	return uart.tail_transmit - uart.head_transmit - 1; // -1:不可以填满。
}

// 获取发送缓冲区中的数据长度。
int32_t uart_transmitValidBufferSize(void)
{
	if(uart.tail_transmit <= uart.head_transmit)
		return uart.head_transmit - uart.tail_transmit;
	return uart.head_transmit + (UART_BUFFER_SIZE_TRANSMIT - uart.tail_transmit);
}

// 从发送缓冲区取出一个字节，发送出去。
// 不检查缓冲区是否空，不检查发送状态。
void uart_transmitAByte(void)
{
   //USART_SendData(USART3,uart.buffer_transmit[uart.tail_transmit]);
		uart.tail_transmit ++;
		if(uart.tail_transmit >= UART_BUFFER_SIZE_TRANSMIT)
		uart.tail_transmit = 0;
}

// 把一段数据放入发送缓冲区。
// 返回值：{uart_SUCCEED,uart_FAILED}
uint8_t uart_transmit(const void * data,uint32_t len)
{
	int i;
	if(uart_transmitIdleBufferSize() < len)
	{
		return UART_FAILED; // 空间不够。
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
	// 如果发送没有正在进行，就启动发送。
	//if(USART_GetFlagStatus(USART3,USART_FLAG_TXE) == SET)
        uart_transmitAByte();
	//
	return UART_SUCCEED;
}

// 获取接收缓冲区中的数据长度。
int32_t uart_receiveValidBufferSize(void)
{
	if(uart.tail_receive <= uart.head_receive)
		return uart.head_receive - uart.tail_receive;
	return uart.head_receive + (UART_BUFFER_SIZE_RECEIVE - uart.tail_receive);
}

// 把一个字节放入接收缓冲区。
// 缓冲区满则失败。
// 返回值：{uart_SUCCEED,uart_FAILED}。
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

// 从接收缓冲区取出数据，返回取出的长度。
// 取出的长度为outputBufferLength和uart_receiveValidBufferSize()的最小者。
int32_t uart_readReceiveBuffer(void * outputBuffer,int32_t outputBufferLength)
{
    // 计算outputBufferLength和uart_receiveValidBufferSize()的最小值。
    uint32_t returnLength = uart_receiveValidBufferSize();
	  uint32_t i;
    if(outputBufferLength < returnLength)
        returnLength = outputBufferLength;
    //
    // 复制数据，推进指针。
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


