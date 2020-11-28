#ifndef __LED_H
#define __LED_H
#include "sys.h"
//LED ¶Ë¿Ú¶¨Òå
#define LED PAout(8) // PA8
void LED_Init(void); 
void Led_Flash(u16 time);
#endif
