#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
u8 Way_Angle=1;                             //��ȡ�Ƕȵ��㷨��1����Ԫ�� 
u8 Flag_Qian,Flag_Hou,Flag_Left,Flag_Right,Flag_sudu=2; //����ң����صı���
u8 Flag_Stop=1,Flag_Show=1;                 //ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
int Encoder_Left,Encoder_Right;             //���ұ��������������
int Moto1,Moto2;                            //���PWM���� Ӧ��Motor�� ��Moto�¾�	
int Temperature;                            //��ʾ�¶�
int Voltage;                                //��ص�ѹ������صı���
float Angle_Balance,Gyro_Balance,Gyro_Turn; //ƽ����� ƽ�������� ת��������
float Show_Data_Mb;                         //ȫ����ʾ������������ʾ��Ҫ�鿴������
u32 Distance;                               //���������
u8 delay_50,delay_flag,Bi_zhang=0,PID_Send,Flash_Send;   //Ĭ�������
float Acceleration_Z;                       //Z����ٶȼ�  
float PA=100,PB=500,PC=888,PD=6813;//PID����
u16 PID_Parameter[10],Flash_Parameter[10];  //Flash�������
int main(void)
{ 
	Stm32_Clock_Init(9);            //=====ϵͳʱ������
	delay_init(72);                 //=====��ʱ��ʼ��
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	LED_Init();                     //=====��ʼ���� LED ���ӵ�Ӳ���ӿ�
	KEY_Init();                     //=====������ʼ��
	OLED_Init();                    //=====OLED��ʼ��
	uart_init(72,128000);           //=====��ʼ������1
  uart2_init(36,9600);            //=====����3��ʼ��
  MiniBalance_PWM_Init(7199,0);   //=====��ʼ��PWM 10KHZ������������� �����ʼ������ӿ� ��ΪMiniBalance_PWM_Init(9999,35) 200HZ
	Encoder_Init_TIM2();            //=====�������ӿ�
	Encoder_Init_TIM3();            //=====��ʼ��������2 
	IIC_Init();                     //=====ģ��IIC��ʼ��
  MPU6050_initialize();           //=====MPU6050��ʼ��	
  DMP_Init();                     //=====��ʼ��DMP     
	Flash_Read();                   //=====��ȡPID����
 	EXTI_Init();                    //=====MPU6050 5ms��ʱ�жϳ�ʼ��
	while(1)
		{     
				 if(Flash_Send==1)          //д��PID������Flash,��app���Ƹ�ָ��
					{
          	Flash_Write();	
						Flash_Send=0;	
					}
					if(Flag_Show==0)          //ʹ��MiniBalance APP��OLED��ʾ��
					{
						APP_Show();	
						oled_show();             //===��ʾ����
					}
					else                      //ʹ��MiniBalance��λ�� ��λ��ʹ�õ�ʱ����Ҫ�ϸ��ʱ�򣬹ʴ�ʱ�ر�app��ز��ֺ�OLED��ʾ��
					{
				      DataScope();          //����MiniBalance��λ��
					}	
				  delay_flag=1;	
					delay_50=0;
					while(delay_flag);	     //ͨ��MPU6050��INT�ж�ʵ�ֵ�50ms��׼��ʱ		
		} 
}
