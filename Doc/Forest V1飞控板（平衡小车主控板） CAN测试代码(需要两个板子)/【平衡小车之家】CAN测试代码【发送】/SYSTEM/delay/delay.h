#ifndef __DELAY_H
#define __DELAY_H 			   
#include <stm32f10x_map.h>
#include <stm32f10x_nvic.h>
//作者：平衡小车之家
//我的淘宝小店：http://shop114407458.taobao.com/
//版权所有 盗版必究
void delay_init(u8 SYSCLK);
void delay_ms(u16 nms);
void delay_us(u32 nus);

#endif





























