#ifndef	__LED_H
#define __LED_H
#include "sys.h"
#define led_r  PGout(14)
#define led_l  PDout(13)
void LED_Init(void);
void led_test(void);

#endif
