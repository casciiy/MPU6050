#include "motor.h"
void MiniBalance_Motor_Init(void)
{
	RCC->APB2ENR|=1<<2;       //PORTA时钟使能   
	RCC->APB2ENR|=1<<3;       //PORTB时钟使能   
	GPIOB->CRL&=0X0000FFFF;   //PORTB12 13 14 15推挽输出
	GPIOB->CRL|=0X22220000;   //PORTB12 13 14 15推挽输出
	
	GPIOB->CRH&=0XFFFFFF00;   //PORTB12 13 14 15推挽输出
	GPIOB->CRH|=0X00000022;   //PORTB12 13 14 15推挽输出
	
	GPIOA->CRL&=0XFFFF0000;   //PORTB12 13 14 15推挽输出
	GPIOA->CRL|=0X00002222;   //PORTB12 13 14 15推挽输出
	
	GPIOA->CRH&=0XF00FFFFF;   //PORTB12 13 14 15推挽输出
	GPIOA->CRH|=0X02200000;   //PORTB12 13 14 15推挽输出
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	MiniBalance_Motor_Init(); //初始化电机控制所需IO
//	RCC->APB1ENR|=1<<2;       //TIM4时钟使能    
//	RCC->APB2ENR|=1<<3;       //PORTB时钟使能     
//	GPIOB->CRH&=0XFFFFFF00;   //PORTB0 1复用输出
//	GPIOB->CRH|=0X000000BB;   //PORTB0 1复用输出
//	TIM4->ARR=arr;//设定计数器自动重装值 
//	TIM4->PSC=psc;//预分频器不分频
//	TIM4->CCMR2|=6<<12;//CH4 PWM1模式	
//	TIM4->CCMR2|=6<<4; //CH3 PWM1模式	
//	TIM4->CCMR2|=1<<11;//CH4预装载使能	 
//	TIM4->CCMR2|=1<<3; //CH3预装载使能	  
//	TIM4->CCER|=1<<12; //CH4输出使能	   
//	TIM4->CCER|=1<<8;  //CH3输出使能	
//	TIM4->CR1=0x8000;  //ARPE使能 
//	TIM4->CR1|=0x01;   //使能定时器3 										  
} 

