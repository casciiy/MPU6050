#include "sys.h"
  /**************************************************************************
作者：平衡小车之家
我的淘宝小店：http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                              //获取角度的算法，1:DMP
u8 Flag_Show=0;                             //显示标志位
int Encoder_Left,Encoder_Right;             //左右编码器的脉冲计数
int Temperature;                            //显示温度
float Angle_Balance,Angle_Y;                        //倾角
float Show_Data;                         //全局显示变量，用于显示需要查看的数据
int main(void)
{ 
	Stm32_Clock_Init(9);            //系统时钟设置
	delay_init(72);                 //延时初始化
	JTAG_Set(JTAG_SWD_DISABLE);     //=====关闭JTAG接口
	JTAG_Set(SWD_ENABLE);           //=====打开SWD接口 可以利用主板的SWD接口调试
	LED_Init();                     //初始化与 LED 连接的硬件接口
	KEY_Init();                     //按键初始化
	OLED_Init();                    //OLED初始化
	uart_init(72,9600);             //初始化串口1
	NRF24L01_Init();                //=====NRF24L01模块初始化
  NRF24L01_FindMyself();          //=====NRF24L01自检程序 没通过(没有接无线模块)就死在这是闪灯
	while(1)
		{
			NRF24L01();//发送模式
		} 
}
