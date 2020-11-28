#include <stm32f10x_map.h>
//#include <stm32f10x_nvic.h>   
#include "led.h"
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//版权所有 盗版必究
void LED_N_KEY_Init(void)
{
	/////////////////////////////////////LED初始化////////////////////////////////////
	RCC->APB2ENR|=1<<2;    //使能PORTA时钟	   	 
	GPIOA->CRH&=0XFFFFFFF0; 
	GPIOA->CRH|=0X00000003;//PA8 推挽输出   	 
  GPIOA->ODR|=1<<8;
}






