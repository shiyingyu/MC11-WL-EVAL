#include "debug_usart.h"
#include "usart.h"
#include <stdio.h>

// ȷ��û�д� C ������ʹ�ð������ĺ���
// #pragma import(__use_no_semihosting)

// ����_sys_exit()�Ա���ʹ�ð�����ģʽ
int _sys_exit(int x)
{
	x = x;
}

// ��׼����Ҫ��֧�ֺ���
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
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&CG_HUART_DEBUG, (uint8_t *)&ch, 1, 0xFFFF);
  return ch;
}

/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&CG_HUART_DEBUG, &ch, 1, 0xFFFF);
  return ch;
}
