#include "sys.h"
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//��Ȩ���� ����ؾ�
u8 res,canbuf[8];
int main(void)
{		
	Stm32_Clock_Init(9);        //ϵͳʱ������
	uart_init(72,9600);	 	      //���ڳ�ʼ�� 
	delay_init(72);	            //��ʱ��ʼ��
	LED_N_KEY_Init();		  	    //��ʼ����LED���ӵ�Ӳ���ӿ�
	delay_ms(200);
	OLED_Init();               //OLED��ʼ��
	CAN_Mode_Init(1,2,3,6,0);  //CAN��ʼ��
	LED1=0;
	while(1)
	{ 
	  	CAN_SEND(0X200,6,6,6,6,6,6,6,6);//CAN����
  }
 }	 








