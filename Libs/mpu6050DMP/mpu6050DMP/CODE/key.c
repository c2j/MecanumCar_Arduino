#include <stm32f10x_lib.h>
#include "key.h"	 //c13 e0

void KEY_Init(void)
{
	RCC->APB2ENR|=0X00000050;	 //ebable port c e clock
	GPIOE->CRL  &=0XFFFFFFF8;
	GPIOE->CRL  |=0X00000008;
	GPIOC->CRH  &=0XFFF8FFFFF;
	GPIOC->CRH  |=0X000800000;
	GPIOE->ODR|=1<<0;	      //SET ITPUT PULL UP;
	GPIOC->ODR|=1<<13;
}

