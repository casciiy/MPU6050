#include "sys.h"
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//��Ȩ���� ����ؾ�
u8 CAN_Receive;
int main(void)
{		
	Stm32_Clock_Init(9);        //ϵͳʱ������
	delay_init(72);	            //��ʱ��ʼ��
	uart_init(72,9600);	 	      //���ڳ�ʼ�� 
	LED_N_KEY_Init();		  	    //��ʼ����LED���ӵ�Ӳ���ӿ�
	delay_ms(200);
	CAN_Mode_Init(1,2,3,6,0);  //CAN��ʼ��
	OLED_Init();               //OLED��ʼ��

	while(1)
	{ 
													 OLED_ShowString(00,20,"Receive");
		 OLED_ShowNumber(65,20,CAN_Receive,5,12);
		 if(CAN_Receive==6)		 OLED_ShowString(00,30,"Normal");
		 else		               OLED_ShowString(00,30,"Failed");	 
		                       OLED_ShowString(00,40,"Reception");
		 OLED_Refresh_Gram();	
		 printf("���յ������ݣ�  %d\r\n",CAN_Receive);
		 if(CAN_Receive==6)		  printf("Normal\r\n");
		 else		                printf("Failed\r\n");
     delay_ms(200);		
		 LED1=~ LED1;
  }
}	 








