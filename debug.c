#include "common.h"
//#include "UART_user.h"
#include "utils/uartstdio.h"
#include "utils/ustdlib.h"
#include <stdarg.h>


/* Private define ------------------------------------------------------------*/
#define DEBUG_BUFFER_SIZE 1000


/* Private typedef -----------------------------------------------------------*/



/* Private variables ---------------------------------------------------------*/
char debug_buffer[DEBUG_BUFFER_SIZE];

/* Private function prototypes ---------------------------------------------*/



/* Private functions ---------------------------------------------------------*/

void debug_printf(const char *fmt, ...)
{
    __va_list argptr;//VA_LIST ����C�����н����������һ���
    va_start(argptr, fmt);//VA_START�꣬��ȡ�ɱ�����б�ĵ�һ�������ĵ�ַ��ap������Ϊva_list��ָ�룬v�ǿɱ��������ߵĲ�������
    vsprintf(debug_buffer, fmt, argptr);
    va_end(argptr);//VA_END�꣬���va_list�ɱ�����б�
    UARTprintf(debug_buffer);
}


