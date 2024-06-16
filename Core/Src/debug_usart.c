#include "debug_usart.h"
#include "usart.h"
#include <stdio.h>

// 确保没有从 C 库链接使用半主机的函数
// #pragma import(__use_no_semihosting)

// 定义_sys_exit()以避免使用半主机模式
int _sys_exit(int x)
{
	x = x;
}

// 标准库需要的支持函数
struct __FILE
{
	int handle;
	
	/* Whatever you require here. If the only file you are using is */
	/* standard output using printf() for debugging, no file handling */
	/* is required. */
};

/* FILE is typedef'd in stdio.h. */
FILE __stdout;


/**
  * 函数功能: 重定向c库函数printf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&CG_HUART_DEBUG, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * 函数功能: 重定向c库函数getchar,scanf到DEBUG_USARTx
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明：无
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&CG_HUART_DEBUG, &ch, 1, 0xFFFF);
  return ch;
}
