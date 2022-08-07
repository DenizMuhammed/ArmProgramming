/*
 * Arduino.h
 *
 *  Created on: 20 Ekim 2020
 *      Author: muhammed deniz
 */

#ifndef ARDUINO_H_
#define ARDUINO_H_
#endif /* ARDUINO_H_ */

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

#define RCC ((RCC_typedef *) 0x40023800)     // 0x40023800= ABSULUTE_ADDRESS_USART2
#define GPIO_base ((GPIO_typedef *) 0X40020000)
#define GPIO_offset ((GPIO_typedef *) 0X400)
typedef __UINT8_TYPE__ uint8;

typedef enum {INPUT=0,OUTPUT=1} pinOI_t;
typedef enum {PA=0,PB=1,PC=2,PD=3,PE=4,PF=5,PG=6,PH=7,PI=8} portName_t;
typedef enum {LOW=0,HIGH=1} pinState_t;



void pinMode(portName_t port, uint8 portno, pinOI_t oi ) // pinMode(PA,3,OUTPUT);
{
 RCC->AHB1ENR|=1<<port; // ilgili portun clock sinyali açýldý.
 if (oi==OUTPUT)
 {
   ((GPIO_base)+(GPIO_offset)*port)->MODER&=~(0b11<<(portno*2)); // clear
   ((GPIO_base)+(GPIO_offset)*port)->MODER|=0b01<<(portno*2);  //output
 }

 else if(oi==INPUT)
 {
	 ((GPIO_base)+(GPIO_offset)*port)->MODER&=~(0b11<<(portno*2)); // clear and input
 }
}

void digitalWrite(portName_t port, uint8 portno, pinState_t state)
{
	if(state==HIGH)
	((GPIO_base)+(GPIO_offset)*port)->ODR|=1<<portno;
	else if(state==LOW)
	((GPIO_base)+(GPIO_offset)*port)->ODR&=~(1<<portno);
}


void delay(__UINT16_TYPE__ saniye)
{
    __UINT32_TYPE__ salise;
	for(;saniye>0;salise=10000000,saniye--)
    while(salise--);
}
