#include "stm32f4xx.h"

typedef struct
{
  volatile  __UINT32_TYPE__ MODER;    /*!< GPIO port mode register,               Address offset: 0x00      */
  volatile  __UINT32_TYPE__ OTYPER;   /*!< GPIO port output type register,        Address offset: 0x04      */
  volatile  __UINT32_TYPE__ OSPEEDR;  /*!< GPIO port output speed register,       Address offset: 0x08      */
  volatile  __UINT32_TYPE__ PUPDR;    /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
  volatile  __UINT32_TYPE__ IDR;      /*!< GPIO port input data register,         Address offset: 0x10      */
  volatile  __UINT32_TYPE__ ODR;      /*!< GPIO port output data register,        Address offset: 0x14      */
  volatile  __UINT16_TYPE__ BSRRL;    /*!< GPIO port bit set/reset low register,  Address offset: 0x18      */
  volatile  __UINT16_TYPE__ BSRRH;    /*!< GPIO port bit set/reset high register, Address offset: 0x1A      */
  volatile  __UINT32_TYPE__ LCKR;     /*!< GPIO port configuration lock register, Address offset: 0x1C      */
  volatile  __UINT32_TYPE__ AFR[2];   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
} GPIO_typedef;

#define ABSOLUTE_ADDRESS_PORTD 0x40020C00
#define PORTD ((GPIO_typedef *) ABSOLUTE_ADDRESS_PORTD)

void bekle(__UINT16_TYPE__); // fonksiyon prototipi

int main(void)
{
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);

PORTD->MODER=0x55000000;
PORTD->ODR=0x0000;
while(1)
{
 bekle(3);
 PORTD->ODR=0xF000;
 bekle(3);
 PORTD->ODR=0x0000;
}

} // end_main

void bekle(__UINT16_TYPE__ saniye)
{
    __UINT32_TYPE__ salise;
	for(;saniye>0;salise=10000000,saniye--)
    while(salise--);
}

/*SON*/

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

  return;
}


uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}

