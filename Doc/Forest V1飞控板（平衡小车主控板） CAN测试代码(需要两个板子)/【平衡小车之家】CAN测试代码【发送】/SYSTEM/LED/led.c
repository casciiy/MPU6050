#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>   
#include "led.h"
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//��Ȩ���� ����ؾ�
void LED_N_KEY_Init(void)
{
	/////////////////////////////////////LED��ʼ��////////////////////////////////////
	RCC->APB2ENR|=1<<2;    //ʹ��PORTAʱ��	   	 
	GPIOA->CRH&=0XFFFFFFF0; 
	GPIOA->CRH|=0X00000003;//PA8 �������   	 
  GPIOA->ODR|=1<<8;
}






