#include "TM4C.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include "usart.h"
void InitConsole0(void);
int UART1IntHandler(void);
void InitConsole1(void);
extern int sign;
extern char BUFFER[8];
extern int count_trans;
#define BUFFER_LENGTH  6
