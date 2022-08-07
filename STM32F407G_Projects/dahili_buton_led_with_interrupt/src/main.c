
// mavi led: butona basýnca yanacak. yeþil led 1 sn'de bir yanacak.
/*
 yeþil led: PD12 -> output
 mavi led: PD15 -> output
 user buton: PA0 -> input
 */
#include "stm32f4xx.h"

typedef struct port_nesnesi
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

typedef struct exti_nesnesi
{
	volatile uint32_t IMR;    /*!< EXTI Interrupt mask register,            Address offset: 0x00 */
	volatile uint32_t EMR;    /*!< EXTI Event mask register,                Address offset: 0x04 */
	volatile uint32_t RTSR;   /*!< EXTI Rising trigger selection register,  Address offset: 0x08 */
	volatile uint32_t FTSR;   /*!< EXTI Falling trigger selection register, Address offset: 0x0C */
	volatile uint32_t SWIER;  /*!< EXTI Software interrupt event register,  Address offset: 0x10 */
	volatile uint32_t PR;     /*!< EXTI Pending register,                   Address offset: 0x14 */
} EXTI_typedef;

typedef struct syscfg_nesnesi
{
  volatile   uint32_t MEMRMP;       /*!< SYSCFG memory remap register,                      Address offset: 0x00      */
  volatile   uint32_t PMC;          /*!< SYSCFG peripheral mode configuration register,     Address offset: 0x04      */
  volatile   uint32_t EXTICR[4];    /*!< SYSCFG external interrupt configuration registers, Address offset: 0x08-0x14 */
             uint32_t RESERVED[2];  /*!< Reserved, 0x18-0x1C                                                          */
  volatile   uint32_t CMPCR;        /*!< SYSCFG Compensation cell control register,         Address offset: 0x20      */
} SYSCFG_typedef;

#define ABSOLUTE_ADDRESS_EXTI 0x40013C00
#define ABSOLUTE_ADDRESS_SYSCFG 0x40013800
#define ABSOLUTE_ADDRESS_PORTA 0x40020000
#define ABSOLUTE_ADDRESS_PORTD 0x40020C00

#define yesil 12
#define mavi 15
#define buton 0
#define loop 1
//NESNELER:
#define Exti ((EXTI_typedef *) ABSOLUTE_ADDRESS_EXTI)
#define Syscfg ((SYSCFG_typedef *) ABSOLUTE_ADDRESS_SYSCFG)
#define PORTA ((GPIO_typedef *) ABSOLUTE_ADDRESS_PORTA)
#define PORTD ((GPIO_typedef *) ABSOLUTE_ADDRESS_PORTD)


void PAPD_ayar(void); // port ve pinleri konfigürasyon ayarlarý için
void bekle(__UINT16_TYPE__);
void Peripheral_Interrupt_ayar(void);
void NVIC_ayar(void);

int main(void)
{
/* SETUP */
/*NOT: saat ayarýný fonksiyonun içerisinde yapýnca kodun çalýþmaya baþlamasý gecikiyor*/
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG,ENABLE);

PAPD_ayar();
Peripheral_Interrupt_ayar();
NVIC_ayar();

/* END OF SETUP */
while(loop)
{

	PORTD->ODR |=1<<yesil; // yeþil ledi yak. digitalWrite(6,HIGH);
	bekle(5); // delay(5000);
	PORTD->ODR &=~(1<<yesil); // yeþil ledi söndür.
	bekle(5);


	/*if(((PORTA->IDR) & 0b1)==1)
	{
		PORTD->ODR |=1<<mavi; // mavi ledi yak.

	}

	else
	{
		PORTD->ODR &=~(1<<mavi); // mavi ledi söndür.
	} */
}


} /* End of main */

void PAPD_ayar(void)
{

PORTD->MODER|=0b01<<30; // mavi led (PD15) output
PORTD->MODER|=0b01<<24; // yeþil led (PD12) output

PORTA->MODER|=0b0; // buton pin (PA0) input

}

void Peripheral_Interrupt_ayar(void)
{
Syscfg->EXTICR[0]&=~(0b1111<<0); // EXTI0 hattýna PA0 baðlandý.
Exti->IMR|=0b1<<0; // EXTI0 hattýndan oluþan kesme maskesi kaldýrýldý.
Exti->FTSR|=0b1<<0; // EXTI0 hattýnda düþen kenar tetiklemesi ile kesme oluþacak.
}

void NVIC_ayar(void)
{
	NVIC_SetPriority(EXTI0_IRQn,1); //  hat / ve hattýn önceliði (EXTI0_IRQn=6)
	NVIC_EnableIRQ(EXTI0_IRQn); // hattan gelen isteðin NVIC tarafýndan kabul edilmesi için.
}

void bekle(__UINT16_TYPE__ saniye)
{
    __UINT32_TYPE__ salise;
	for(;saniye>0;salise=10000000,saniye--)
    while(salise--);
}

void EXTI0_IRQHandler(void) // kesme fonksiyonu
{
	Exti->PR|=0b1; // pending biti set edilerek temizlendi.
	PORTD->ODR^=1<<mavi; // toogle iþlemi
}


/*SON*/

void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){

  return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){

  return -1;
}
