
// PERIPHERAL AND CLOCK CONFIGURATION ADJUSTMENT AT REGISTER LEVEL FOR USART2 ONLY TRANSMITTER MODE (HALF-DUPLEX MODE)
// NO USE OF INTERRUPT

void bekle(__UINT16_TYPE__); // fonksiyon prototipi
void Setup(void); // fonksiyon prototipi

#define Loop() while(1)
#define PORTA ((GPIO_typedef *) 0x40020000)  // 0x40020000= ABSOLUTE_ADDRESS_PORTA
#define UART2 ((UART2_typedef *) 0x40004400) // 0x40004400= ABSULUTE_ADDRESS_UART2
#define RCC ((RCC_typedef *) 0x40023800)     // 0x40023800= ABSULUTE_ADDRESS_USART2

typedef struct
{
  volatile  __UINT32_TYPE__ MODER;    //!< GPIO port mode register,
  volatile  __UINT32_TYPE__ OTYPER;   //!< GPIO port output type register,
  volatile  __UINT32_TYPE__ OSPEEDR;  //!< GPIO port output speed register,
  volatile  __UINT32_TYPE__ PUPDR;    //!< GPIO port pull-up/pull-down register,
  volatile  __UINT32_TYPE__ IDR;      //!< GPIO port input data register,
  volatile  __UINT32_TYPE__ ODR;      //!< GPIO port output data register,
  volatile  __UINT16_TYPE__ BSRRL;    //!< GPIO port bit set/reset low register,
  volatile  __UINT16_TYPE__ BSRRH;    //!< GPIO port bit set/reset high register,
  volatile  __UINT32_TYPE__ LCKR;     //!< GPIO port configuration lock register,
  volatile  __UINT32_TYPE__ AFR[2];   //!< GPIO alternate function registers,
} GPIO_typedef;

typedef struct
{
  volatile __UINT16_TYPE__ SR;         //!< USART Status register
	       __UINT16_TYPE__ RESERVED0;  //!< Reserved, 0x02
  volatile __UINT16_TYPE__ DR;         //!< USART Data register
           __UINT16_TYPE__ RESERVED1;  //!< Reserved, 0x06
  volatile __UINT16_TYPE__ BRR;        //!< USART Baud rate register
           __UINT16_TYPE__ RESERVED2;  //!< Reserved, 0x0A
  volatile __UINT16_TYPE__ CR1;        //!< USART Control register 1
           __UINT16_TYPE__ RESERVED3;  //!< Reserved, 0x0E
  volatile __UINT16_TYPE__ CR2;        //!< USART Control register 2
           __UINT16_TYPE__ RESERVED4;  //!< Reserved, 0x12
  volatile __UINT16_TYPE__ CR3;        //!< USART Control register 3
           __UINT16_TYPE__ RESERVED5;  //!< Reserved, 0x16
  volatile __UINT16_TYPE__ GTPR;       //!< USART Guard time and prescaler register
           __UINT16_TYPE__ RESERVED6;  //!< Reserved, 0x1A
} UART2_typedef;

typedef struct
{
  volatile __UINT32_TYPE__ CR;            //!< RCC clock control register,
  volatile __UINT32_TYPE__ PLLCFGR;       //!< RCC PLL configuration register,
  volatile __UINT32_TYPE__ CFGR;          //!< RCC clock configuration register,
  volatile __UINT32_TYPE__ CIR;           //!< RCC clock interrupt register,
  volatile __UINT32_TYPE__ AHB1RSTR;      //!< RCC AHB1 peripheral reset register,
  volatile __UINT32_TYPE__ AHB2RSTR;      //!< RCC AHB2 peripheral reset register,
  volatile __UINT32_TYPE__ AHB3RSTR;      //!< RCC AHB3 peripheral reset register,
           __UINT32_TYPE__ RESERVED0;     //!< Reserved, 0x1C
  volatile __UINT32_TYPE__ APB1RSTR;      //!< RCC APB1 peripheral reset register,
  volatile __UINT32_TYPE__ APB2RSTR;      //!< RCC APB2 peripheral reset register,
           __UINT32_TYPE__ RESERVED1[2];  //!< Reserved, 0x28-0x2C
  volatile __UINT32_TYPE__ AHB1ENR;       //!< RCC AHB1 peripheral clock register,
  volatile __UINT32_TYPE__ AHB2ENR;       //!< RCC AHB2 peripheral clock register,
  volatile __UINT32_TYPE__ AHB3ENR;       //!< RCC AHB3 peripheral clock register,
           __UINT32_TYPE__ RESERVED2;     //!< Reserved, 0x3C
  volatile __UINT32_TYPE__ APB1ENR;       //!< RCC APB1 peripheral clock enable register,
  volatile __UINT32_TYPE__ APB2ENR;       //!< RCC APB2 peripheral clock enable register,
           __UINT32_TYPE__ RESERVED3[2];  //!< Reserved, 0x48-0x4C
  volatile __UINT32_TYPE__ AHB1LPENR;     //!< RCC AHB1 peripheral clock enable in low power mode register
  volatile __UINT32_TYPE__ AHB2LPENR;     //!< RCC AHB2 peripheral clock enable in low power mode register
  volatile __UINT32_TYPE__ AHB3LPENR;     //!< RCC AHB3 peripheral clock enable in low power mode register
           __UINT32_TYPE__ RESERVED4;     //!< Reserved, 0x5C
  volatile __UINT32_TYPE__ APB1LPENR;     //!< RCC APB1 peripheral clock enable in low power mode register
  volatile __UINT32_TYPE__ APB2LPENR;     //!< RCC APB2 peripheral clock enable in low power mode register
           __UINT32_TYPE__ RESERVED5[2];  //!< Reserved, 0x68-0x6C
  volatile __UINT32_TYPE__ BDCR;          //!< RCC Backup domain control register
  volatile __UINT32_TYPE__ CSR;           //!< RCC clock control & status register
           __UINT32_TYPE__ RESERVED6[2];  //!< Reserved, 0x78-0x7C
  volatile __UINT32_TYPE__ SSCGR;         //!< RCC spread spectrum clock generation register
  volatile __UINT32_TYPE__ PLLI2SCFGR;    //!< RCC PLLI2S configuration register

} RCC_typedef;

int main(void)
{

Setup();

unsigned char veri;

Loop()
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

void Setup(void)
{

/* Clock Config.*/

// Source Confg.
RCC->CR|=1<<16; // HSE source ON
while(!(RCC->CR & 1<<17)); // if HSERDY!=1 wait
RCC->CR|=1<<19; // CSSON=1
RCC->CR|=1<<24; // PLLON=1
while(!(RCC->CR & 1<<25)); // if PLLRDY!=1 wait

// PLL Confg.
RCC->PLLCFGR|=1<<22; // PLL source HSE ( PLL Source Mux HSE Selected)
RCC->PLLCFGR|=0x00000000; // PLL_P=2
RCC->PLLCFGR|=0b101010000000000; // PLL_N=336
RCC->PLLCFGR|=0b001000; // PLL_M=8

// Prescale Confg.
RCC->CFGR|=1<<1; // PLL_Clock=active (System Clock Mux PLLCLK Selected)
RCC->CFGR|=0x0000; // AHB Prescaler =1
RCC->CFGR|=0b101<<10; // APB1 Prescaler=4
RCC->CFGR|=0b100<<13; // APB2 Prescaler=2

//Interrupt Flag Config.
RCC->CIR|=1<<19; // HSE Ready Flag Clear
RCC->CIR|=1<<23; // CSS Flag Clear
RCC->CIR|=1<<20; // PLL Ready Flag Clear

//Peripherals Clock Enable Config. ( RCC And PORTA)
RCC->AHB1ENR|=0x00000001; // PORTA Clock Enable
RCC->APB1ENR|=1<<17; // USART2 Clock Enable

//bekle(1);

/* Peripherals Config.*/

PORTA->MODER|=0x00000020; //PA2 alternate mod olarak ayarlandý.
PORTA->OTYPER|=1<<2; //open drain ayarlandý NEDEN ? ÝNCELE !
//PORTA->PUPDR|=1<<6; // Pull-Up ayarlandý. NEDEN ? ÝNCELE !
PORTA->OSPEEDR|=0x00000030; // very high speed ayarlandý
PORTA->AFR[0]|=0x00000700; // PA3 için AF7(Usart2_Tx) ayarlandý.

UART2->CR1|=1<<13; // UE biti 1 yapýldý.
UART2->CR1|=1<<3; // TE biti 1 yapýldý. Transmit aktif.
UART2->CR3|=1<<3; // HDSEL biti 1 yapýldý. Half-duplex mod aktif !
UART2->CR1|=0x00; // M biti 8 bit iletisim için 0 olarak býrakýldý
//42MHz de 9600baud/rate icin: BRR=0d273.4375 ise fraction:0111, mantissa:0001 0001 0001
UART2->BRR|=0x00001117; // for 9600 baud/rate
//bekle(1);
}

/*SON*/
