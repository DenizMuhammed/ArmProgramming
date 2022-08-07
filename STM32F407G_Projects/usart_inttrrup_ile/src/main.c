/* AÇIKLAMA:
 * normal çalýþmada PA4 e baðlý bir led belirli aralýklarla yanýp sönüyor.
 * Eðer PA3 RX hattýndan bir data gelirse hemen kesme fonksiyonuna dallanýlýyor ve buradaki komutlara göre
 * PA4 e baðlý led yakýlýyor veya söndürülüyor.
 **/

#include "stm32f4xx.h"

void bekle(__UINT16_TYPE__); // fonksiyon prototipi
void PORTA_ayar(void);
void USART2_ayar(void);
void NVIC_ayar(void);

typedef struct gpio_nesnesi
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

typedef struct usart2_nesnesi
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
} USART2_typedef;

#define TX  2
#define RX  3
#define LED 4
#define PA3 RX
#define PA2 TX
#define PA4 LED

#define ABSOLUTE_ADDRESS_PORTA (0x40020000)
#define ABSOLUTE_ADDRESS_USART2 (0x40004400)

/* Nesneler: */
#define PORTA ((GPIO_typedef *) ABSOLUTE_ADDRESS_PORTA)
#define UART2 ((USART2_typedef *) ABSOLUTE_ADDRESS_USART2)
unsigned char veri;

int main(void)
{

RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //PORTA Clock Enable
RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE); //USART2 Clock Enable

PORTA_ayar();
USART2_ayar();
NVIC_ayar();

while(1)
{
bekle(5);
PORTA->ODR|=1<<LED;
bekle(5);
PORTA->ODR &=~(1<<LED);
bekle(5);
}


} // End of main


void PORTA_ayar(void)
{
/*
PA2: AF7 USART2_TX
PA3: AF7 USART2_RX
PA4: OUTPUT LED
*/

PORTA->MODER|=0b1010<<4; // PA2 ve PA3 alternatif fonksiyon olarak ayarlandý.
PORTA->MODER|=0b01<<8; // PA4 output olarak ayarlandý. (led için)
PORTA->OSPEEDR|=0b1111<<4; // PA2 ve PA3 hýzý very high yapýldý
PORTA->AFR[0]|=0b01110111<<8; // PA2 AF7 USART2_TX ve PA3 AF7 USART2_RX olarak ayarlandý.
return;
}

void USART2_ayar(void)
{

#define RE 2
#define TE 3
#define RXNEIE 5
#define UE 13
#define RXNE 5

UART2->CR1|=1<<UE; // UE biti 1 yapýldý.
UART2->CR1|=1<<TE|1<<RE; // TE & RE biti 1 yapýldý. Transmit & Reciver Mod Aktif.
UART2->BRR|=0x00001117; // iletiþim hýzý 9600 baud/rate yapýldý.
UART2->CR1|=1<<RXNEIE; // Bir data alýndýðýnda USAR2 kesmesi üretilecek.
return;
}

void NVIC_ayar(void)
{
	NVIC_SetPriority(USART2_IRQn,1); //  hat ve hattýn önceliði (USART2_IRQn=38)
	NVIC_EnableIRQ(USART2_IRQn); // hattan gelen isteðin NVIC tarafýndan kabul edilmesi için.
	return;
}

void USART2_IRQHandler(void) // USART2 kesme fonksiyonu
{

 uint32_t statusRegister=UART2->SR;

 if((statusRegister & (1<<RXNE))!=0)
 {
UART2->SR&=~(1<<RXNE);
veri=UART2->DR;

 if(veri=='y')
 {
	 PORTA->ODR|=1<<LED;
 }

 else if(veri=='s')
 {
	 PORTA->ODR &=~(1<<LED);
 }

 }

}

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
