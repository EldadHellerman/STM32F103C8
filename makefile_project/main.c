#include "stm32f103c8.h"
#include <stdint.h>
//#include "main.h"


int main(void);
volatile char led = 0;
const char string[] = "hello, this is a string which the DMA shold copy to a different location in memory";

void _IRQ27_TIM2(void){
	TIM2->SR &= ~1; //clear interrupt flag
	if(led == 0){
		led = 1;
		GPIOC->ODR &= ~(1<<13); //led on
	}else{
		led = 0;
		GPIOC->ODR |= 1<<13; //led off
	}
}

volatile int i=0;
void delay(int time_us){
	for(i=0; i < time_us*4 + (time_us>>2); i++);
}

void init_clock(void){
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
	RCC->CFGR = RCC_CFGR_PLLSRC  | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLXTPRE_HSE | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 |RCC_CFGR_PPRE2_DIV2;

	RCC->CR |= RCC_CR_HSEON; while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->CR |= RCC_CR_PLLON; while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	RCC->APB2ENR = RCC_APB2ENR_IOPCEN;
	RCC->AHBENR = RCC_AHBENR_DMA1EN;
}

void init_tim2(void){
	TIM2->ARR = 10000 - 1; //Auto reload value
	TIM2->PSC = 7200; //prescaler
  TIM2->DIER |= 1; //enable interrupts
	NVIC->ISER0 |= 0x10000000; //enable timer2 interrupt
}

void dma_init(void){
	DMA->channel[0].CCR = DMA_CCR_MEM2MEM | DMA_CCR_PL_VERY_HIGH | DMA_CCR_PINC | DMA_CCR_MINC;
	DMA->channel[0].CNDTR = 83; //set number of bytes
	DMA->channel[0].CPAR = (int)string; //set src memory address
	DMA->channel[0].CMAR = SRAM_BASE + 0x100; //set dst memory address
}

void dma_start(void){
	DMA->channel[0].CCR |= DMA_CCR_EN; //activate the channel
}

void stk_start(void){
	//maximum of 1.864 sec
	STK->CTRL |= STK_CTRL_ENABLE;
	STK->LOAD = 0xFFFFFF;
	STK->VAL = 0;
}

int stk_stop(void){ //return amount of us since started
	STK->CTRL &= ~STK_CTRL_ENABLE;
	return((0xffffff - STK->VAL)/9);
}

int main(void){
	init_clock();
	init_tim2();
	dma_init();


	stk_start();
	dma_start(); while(!(DMA->ISR & ~DMA_ISR_TCIF1));
	*(int *)(0x20000050) = stk_stop(); 

	GPIOC->CRH = ((uint32_t)(0b0010)) << ((13-8)*4); //set pin 13 to push-pull at high speed (50MHz)
	GPIOC->ODR |= 1<<13; //led off
	TIM2->CR1 |= 1; //CK_INT - write 1 to CEN to enable internal clock

	while(1);
}
