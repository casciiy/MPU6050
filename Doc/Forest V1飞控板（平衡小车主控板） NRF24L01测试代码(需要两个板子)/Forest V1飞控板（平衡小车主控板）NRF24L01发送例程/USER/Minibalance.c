#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                              //��ȡ�Ƕȵ��㷨��1:DMP
u8 Flag_Show=0;                             //��ʾ��־λ
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Temperature;                            //��ʾ�¶�
float Angle_Balance,Angle_Y;                        //���
float Show_Data;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
int main(void)
{ 
	Stm32_Clock_Init(9);            //ϵͳʱ������
	delay_init(72);                 //��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //������ʼ��
	OLED_Init();                    //OLED��ʼ��
	uart_init(72,9600);             //��ʼ������1
	NRF24L01_Init();                //=====NRF24L01ģ���ʼ��
  NRF24L01_FindMyself();          //=====NRF24L01�Լ���� ûͨ��(û�н�����ģ��)��������������
	while(1)
		{
			NRF24L01();//����ģʽ
		} 
}
