#include "sys.h"
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//版权所有 盗版必究
u8 CAN_Receive;
int main(void)
{		
	Stm32_Clock_Init(9);        //系统时钟设置
	delay_init(72);	            //延时初始化
	uart_init(72,9600);	 	      //串口初始化 
	LED_N_KEY_Init();		  	    //初始化与LED连接的硬件接口
	delay_ms(200);
	CAN_Mode_Init(1,2,3,6,0);  //CAN初始化
	OLED_Init();               //OLED初始化

	while(1)
	{ 
													 OLED_ShowString(00,20,"Receive");
		 OLED_ShowNumber(65,20,CAN_Receive,5,12);
		 if(CAN_Receive==6)		 OLED_ShowString(00,30,"Normal");
		 else		               OLED_ShowString(00,30,"Failed");	 
		                       OLED_ShowString(00,40,"Reception");
		 OLED_Refresh_Gram();	
		 printf("接收到的数据：  %d\r\n",CAN_Receive);
		 if(CAN_Receive==6)		  printf("Normal\r\n");
		 else		                printf("Failed\r\n");
     delay_ms(200);		
		 LED1=~ LED1;
  }
}	 








