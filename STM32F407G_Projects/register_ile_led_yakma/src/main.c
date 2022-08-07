// Register ile LED yakma.
#include "stm32f4xx.h"
#include "stm32f4_discovery.h"

void config(void)
{
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE); // D portuna clock saðlayan bus ý aktif etmek için.
	GPIOD->MODER=0b01010101000000000000000000000000; //pinin giris cikis analog alternatif modunu secer
	GPIOD->OSPEEDR=0xFFFFFFFF; // pinin output speed hýzýný secer
	GPIOD->OTYPER=0x00000000; // pinin output tipini secer, push pull/ open drain secer
}

void Delay(uint32_t zaman)
{
	while(zaman--)
	{

	}
}

int main(void)
{

config();
while(1)
{
GPIOD->ODR=0b00000000000000001111000000000000; // 15,14,13,12 pinlerini high yapmak icin.
Delay(16800000);
GPIOD->ODR=0x00000000; // tum pinler low yapildi.
Delay(1680000);
}
}



void EVAL_AUDIO_TransferComplete_CallBack(uint32_t pBuffer, uint32_t Size){
  /* TODO, implement your code here */
  return;
}

uint16_t EVAL_AUDIO_GetSampleCallBack(void){
  /* TODO, implement your code here */
  return -1;
}
