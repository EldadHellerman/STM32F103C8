#include "stm32f103c8.h"
#include <stdint.h>
//#include "main.h"


#define NVIC				0xE000E100



int main(void);
volatile int i=0;
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

void delay(unsigned int value){
	for(i=0; i<value; i++);
	return;
}

int main(void){
	RCC->APB1ENR |= 1; //TIM2 clk enable
	TIM2->ARR = 1000; //Auto reload value
	TIM2->PSC = 72000; //prescaler
	TIM2->CR1 |= 1; //CK_INT - write 1 to CEN to enable internal clock
	TIM2->DIER |= 1; //enable interrupts
	*((uint32_t *)(NVIC)) |= 0x10000000; //enable timer2 interrupt     //NVIC_BASE->ISER[0] |= 0x10000000;
	RCC->APB2ENR |= 0b10000; //turn on GPIOC clock
	RCC->APB2ENR = RCC_APB2ENR_IOPCEN;
	GPIOC->CRH = ((uint32_t)(0b0010)) << ((13-8)*4); //set pin 13 to push-pull at high speed (50MHz)

	GPIOC->ODR |= 1<<13; //led off
	while(1);

	while(1){
		GPIOC->ODR &= ~(1<<13); //led on
		delay(500000);
		GPIOC->ODR |= 1<<13; //led off
		delay(500000);
	}
}
