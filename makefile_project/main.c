#include "stm32f103c8.h"
#include <stdint.h>
//#include "main.h"


int main(void);
volatile char led = 0;

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
void delay(void){
	for(i=0; i<500; i++);
}

void init_clock(void){
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
	delay();
	RCC->CFGR = RCC_CFGR_PLLSRC  | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLXTPRE_HSE | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 |RCC_CFGR_PPRE2_DIV2;
	delay();
	RCC->CR |= RCC_CR_HSEON;
	delay();
	while(!(RCC->CR & RCC_CR_HSERDY));
	delay();
	RCC->CR |= RCC_CR_PLLON;
	delay();
	while(!(RCC->CR & RCC_CR_PLLRDY));

	delay();
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	RCC->APB2ENR = RCC_APB2ENR_IOPCEN;
}

void init_clock_2(void){
	RCC->CR |= RCC_CR_HSEON;
	while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->CFGR = RCC_CFGR_SW_HSE | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV1 |RCC_CFGR_PPRE2_DIV1;
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN;
	RCC->APB2ENR = RCC_APB2ENR_IOPCEN;
}

int main(void){
	init_clock();
	GPIOC->CRH = ((uint32_t)(0b0010)) << ((13-8)*4); //set pin 13 to push-pull at high speed (50MHz)
	TIM2->ARR = 10000 - 1; //Auto reload value
	TIM2->PSC = 7200; //prescaler
	TIM2->CR1 |= 1; //CK_INT - write 1 to CEN to enable internal clock
	TIM2->DIER |= 1; //enable interrupts
	NVIC->ISER0 |= 0x10000000; //enable timer2 interrupt

	GPIOC->ODR |= 1<<13; //led off
	while(1);
}
