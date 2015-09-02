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
    __va_list argptr;//VA_LIST 是在C语言中解决变参问题的一组宏
    va_start(argptr, fmt);//VA_START宏，获取可变参数列表的第一个参数的地址（ap是类型为va_list的指针，v是可变参数最左边的参数）：
    vsprintf(debug_buffer, fmt, argptr);
    va_end(argptr);//VA_END宏，清空va_list可变参数列表：
    UARTprintf(debug_buffer);
}


