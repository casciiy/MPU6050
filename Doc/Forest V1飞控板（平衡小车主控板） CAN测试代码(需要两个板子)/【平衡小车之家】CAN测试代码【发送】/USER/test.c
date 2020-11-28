#include "sys.h"
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//版权所有 盗版必究
u8 res,canbuf[8];
int main(void)
{		
	Stm32_Clock_Init(9);        //系统时钟设置
	uart_init(72,9600);	 	      //串口初始化 
	delay_init(72);	            //延时初始化
	LED_N_KEY_Init();		  	    //初始化与LED连接的硬件接口
	delay_ms(200);
	OLED_Init();               //OLED初始化
	CAN_Mode_Init(1,2,3,6,0);  //CAN初始化
	LED1=0;
	while(1)
	{ 
	  	CAN_SEND(0X200,6,6,6,6,6,6,6,6);//CAN发送
  }
 }	 








