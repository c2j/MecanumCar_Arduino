#include <stm32f10x_lib.h>
#include "led.h"	  //G14 D13
void LED_Init(void)
{
	 RCC->APB2ENR|=0X00000120;   //OPEN IOG IOD CLOCK
	 GPIOG->CRH	&=~0x0c000000;	 //enable g14 out
	 GPIOG->CRH	|= 0x03000000;
	 GPIOD->CRH	&=~0x00C00000;	 //enable d13 out
	 GPIOD->CRH	|= 0x00300000;
	
}

void led_test(void)
{
	led_r=!led_r;
	led_l=!led_l;
}




