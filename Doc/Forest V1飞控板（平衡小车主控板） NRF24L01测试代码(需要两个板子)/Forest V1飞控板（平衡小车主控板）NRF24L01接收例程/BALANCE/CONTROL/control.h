#ifndef __CONTROL_H
#define __CONTROL_H
#include "sys.h"
  /**************************************************************************
���ߣ�ƽ��С��֮��
�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
**************************************************************************/
#define PI 3.14159265
extern	int Balance_Pwm,Velocity_Pwm,Turn_Pwm;
int TIM1_UP_IRQHandler(void);  
void Key(void);
void Get_Angle(u8 way);
void Set_Pwm(int moto1,int moto2);
int myabs(int a);
#endif
