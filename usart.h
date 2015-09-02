#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"

uint8_t uart_transmit(const void * data,uint32_t len);
int32_t uart_receiveValidBufferSize(void);
int32_t uart_readReceiveBuffer(void * outputBuffer,int32_t outputBufferLength);
//
int32_t  uart_transmitIdleBufferSize(void);
int32_t  uart_transmitValidBufferSize(void);
void     uart_transmitAByte(void);
uint8_t  uart_receivePushToBuffer(uint8_t data);

#define UART_BUFFER_SIZE_TRANSMIT 256
#define UART_BUFFER_SIZE_RECEIVE 256
//extern uint8_t uart.buffer_receive[UART_BUFFER_SIZE_RECEIVE];
