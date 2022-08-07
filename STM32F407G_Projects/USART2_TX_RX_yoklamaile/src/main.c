

#include "stm32f4xx.h"



void bekle(__UINT16_TYPE__); // fonksiyon prototipi
typedef struct gpio_peripheral
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

typedef struct uart2_peripheral
{
  volatile __UINT16_TYPE__ SR;         /*!< USART Status register,                   Address offset: 0x00 */
	       __UINT16_TYPE__ RESERVED0;  /*!< Reserved, 0x02                                                */
  volatile __UINT16_TYPE__ DR;         /*!< USART Data register,                     Address offset: 0x04 */
           __UINT16_TYPE__ RESERVED1;  /*!< Reserved, 0x06                                                */
  volatile __UINT16_TYPE__ BRR;        /*!< USART Baud rate register,                Address offset: 0x08 */
           __UINT16_TYPE__ RESERVED2;  /*!< Reserved, 0x0A                                                */
  volatile __UINT16_TYPE__ CR1;        /*!< USART Control register 1,                Address offset: 0x0C */
           __UINT16_TYPE__ RESERVED3;  /*!< Reserved, 0x0E                                                */
  volatile __UINT16_TYPE__ CR2;        /*!< USART Control register 2,                Address offset: 0x10 */
           __UINT16_TYPE__ RESERVED4;  /*!< Reserved, 0x12                                                */
  volatile __UINT16_TYPE__ CR3;        /*!< USART Control register 3,                Address offset: 0x14 */
           __UINT16_TYPE__ RESERVED5;  /*!< Reserved, 0x16                                                */
  volatile __UINT16_TYPE__ GTPR;       /*!< USART Guard time and prescaler register, Address offset: 0x18 */
           __UINT16_TYPE__ RESERVED6;  /*!< Reserved, 0x1A                                                */
} UART2_typedef;

int main(void)
{

#define TX  2
#define RX  3
#define PA3 RX
#define PA2 TX

#define PORTA ((GPIO_typedef *) 0x40020000) // 0x40020000=ABSOLUTE_ADDRESS_PORTA
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //PORTA Clock Enable
PORTA->MODER|=0b10<<6 | 0b10<<4; //  PA3 and PA2 were set as alternative functions.
//PORTA->OTYPER|=0x00000000; //PA3 and PA2 push pull mode
//PORTA->PUPDR|=0b01<<6; // RX pull-up
//PORTA->OTYPER|=1<<TX;  // open drain
//PORTA->OTYPER|=1<<RX;
PORTA->OSPEEDR|=0b11<<4 |0b11<<6; // very high speed ayarlandý
PORTA->AFR[0]|=0b0111<<8 | 0b0111<<12;
#define LED_G 12
#define LED_O 13
#define LED_R 14
#define LED_B 15
#define PORTD ((GPIO_typedef *) 0x40020C00) // 0x40020C00=ABSOLUTE_ADDRESS_PORTD
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); //PORTD Clock Enable
PORTD->MODER|= 0b01010101<<24; // Tüm dahili LED pinleri çýkýþ yapýldý
PORTD->OSPEEDR|=0b11111111<<24; // ALL Internal LED pins High Speed Mode

#define UART2 ((UART2_typedef *) 0x40004400) // 0x40004400= ABSULUTE_ADDRESS_UART2
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); // usart2 bus clock aktif edildi.
UART2->CR1|=1<<13; // UE biti 1 yapýldý.
UART2->CR1|=1<<3|1<<2; // TE & RE biti 1 yapýldý. Transmit & Reciver Mod Aktif.
UART2->BRR|=0x00001117; // for 9600 baud/rate
PORTD->ODR|=0x00000000; //

bekle(1);

unsigned char veri;

bekle(1);
PORTD->ODR|=1<<LED_G;
PORTD->ODR|=1<<LED_O;
PORTD->ODR|=1<<LED_R;
PORTD->ODR|=1<<LED_B;
bekle(2);
PORTD->ODR&=~(0b1111<<12); // LED pinleri temizleniyor.
while(1)
{

/*veri='T';
UART2->DR=veri;
bekle(1);

veri='X';
UART2->DR=veri;
bekle(1); */

//UART2->DR='a';
//bekle(1);
while(!(UART2->SR & 1<<5)); // data gelene kadar bekle

veri=UART2->DR;

if(veri=='o')
{
PORTD->ODR&=~(0b1111<<12); // LED pinleri temizleniyor.
PORTD->ODR|=1<<LED_O;
}

else if(veri=='g')
{
	PORTD->ODR&=~(0b1111<<12); // LED pinleri temizleniyor.
	PORTD->ODR|=1<<LED_G;

}
else if(veri=='r')
{
	PORTD->ODR&=~(0b1111<<12); // LED pinleri temizleniyor.
	PORTD->ODR|=1<<LED_R;
}
else if (veri=='b')
{
	PORTD->ODR&=~(0b1111<<12); // LED pinleri temizleniyor.
	PORTD->ODR|=1<<LED_B;
}
else
{
	PORTD->ODR&=~(0b1111<<12); // LED pinleri temizleniyor.
	PORTD->ODR|=1<<LED_G;
	PORTD->ODR|=1<<LED_O;
	PORTD->ODR|=1<<LED_R;
	PORTD->ODR|=1<<LED_B;

}

}


} // End of main

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
