#include <stm32f10x_lib.h>
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "usmart.h"
#include "IOI2C.h"
#include "dmpctl.h"

int main(void)
{
  Stm32_Clock_Init(9) ;
  delay_init(72);
  uart_init(72,115200); 
  usmart_init(72);
  LED_Init();
  KEY_Init();
  IIC_Init();
  mpu6050_dmp_init();
  while(1)
  {
	 DMP_update();
	 printf("%f   ,   %f   ,   %f    \r\n",Pitch,Roll,Yaw);
	 delay_ms(10);	 
  
  }
	 
}
