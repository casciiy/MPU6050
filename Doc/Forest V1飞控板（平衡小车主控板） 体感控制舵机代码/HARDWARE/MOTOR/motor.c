#include "motor.h"
void MiniBalance_Motor_Init(void)
{
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��   
	GPIOB->CRL&=0X0000FFFF;  
	GPIOB->CRL|=0X22220000;   
}
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
//	MiniBalance_Motor_Init(); //��ʼ�������������IO
	RCC->APB1ENR|=1<<2;       //TIM4ʱ��ʹ��    
	RCC->APB2ENR|=1<<3;       //PORTBʱ��ʹ��  
	GPIOB->CRL&=0X00FFFFFF;   //PORTB6789�������
	GPIOB->CRL|=0XBB000000;   //PORTB6789�������	
	GPIOB->CRH&=0XFFFFFF00;   //PORTB6789�������
	GPIOB->CRH|=0X000000BB;   //PORTB6789�������
	TIM4->ARR=arr;//�趨�������Զ���װֵ 
	TIM4->PSC=psc;//Ԥ��Ƶ������Ƶ
	TIM4->CCMR2|=6<<12;//CH4 PWM1ģʽ	
	TIM4->CCMR2|=6<<4; //CH3 PWM1ģʽ	
	TIM4->CCMR1|=6<<12;//CH2 PWM1ģʽ	
	TIM4->CCMR1|=6<<4; //CH1 PWM1ģʽ	
	TIM4->CCMR2|=1<<11;//CH4Ԥװ��ʹ��	 
	TIM4->CCMR2|=1<<3; //CH3Ԥװ��ʹ��	  
	TIM4->CCMR1|=1<<11;//CH2Ԥװ��ʹ��	 
	TIM4->CCMR1|=1<<3; //CH1Ԥװ��ʹ��	  
	TIM4->CCER|=1<<12; //CH4���ʹ��	   
	TIM4->CCER|=1<<8;  //CH3���ʹ��	
	TIM4->CCER|=1<<4; //CH4���ʹ��	   
	TIM4->CCER|=1<<0;  //CH3���ʹ��	
	TIM4->CR1=0x80;  //ARPEʹ�� 
	TIM4->CR1|=0x01;   //ʹ�ܶ�ʱ�� 				
  TIM4->CCR1= 750;		
	TIM4->CCR2= 750;			 
	TIM4->CCR3= 750;			 
	TIM4->CCR4= 750;		
} 

