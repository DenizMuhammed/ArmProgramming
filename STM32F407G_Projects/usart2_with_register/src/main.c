// SADECE TX MODUNDA

#include "stm32f4xx.h"


void bekle(__UINT16_TYPE__); // fonksiyon prototipi
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

typedef struct
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

#define PORTA ((GPIO_typedef *) 0x40020000) // 0x40020000=ABSOLUTE_ADDRESS_PORTA
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); // usart2 TX için PA3 pini clock aktif edildi.
PORTA->MODER|=0x00000020; //PA2 alternate mod olarak ayarlandý.
PORTA->OTYPER|=1<<2; //open drain ayarlandý NEDEN ? ÝNCELE !
//PORTA->PUPDR|=1<<6; // Pull-Up ayarlandý. NEDEN ? ÝNCELE !
PORTA->OSPEEDR|=0x00000030; // very high speed ayarlandý
PORTA->AFR[0]|=0x00000700; // PA3 için AF7(Usart2_Tx) ayarlandý.

#define UART2 ((UART2_typedef *) 0x40004400) // 0x40004400= ABSULUTE_ADDRESS_UART2
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); // usart2 bus clock aktif edildi.
UART2->CR1|=1<<13; // UE biti 1 yapýldý.
UART2->CR1|=1<<3; // TE biti 1 yapýldý. Transmit aktif.
UART2->CR3|=1<<3; // HDSEL biti 1 yapýldý. Half-dublex mod aktif !
/*UART->CR1|=0x00 M biti 8 bit iletisim için 0 olarak býrakýldý*/
// 42MHz de 9600baud/rate icin: BRR=0d273.4375 ise fraction:0111, mantissa:0001 0001 0001
UART2->BRR|=0x00001117; // for 9600 baud/rate


bekle(1);

unsigned char veri;

while(1)
{
veri='T';
UART2->DR=veri;
bekle(1);

veri='X';
UART2->DR=veri;
bekle(1);
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
