#ifndef __DELAY_H
#define __DELAY_H 			   
#include <stm32f10x_map.h>
#include <stm32f10x_nvic.h>
//���ߣ�ƽ��С��֮��
//�ҵ��Ա�С�꣺http://shop114407458.taobao.com/
//��Ȩ���� ����ؾ�
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























