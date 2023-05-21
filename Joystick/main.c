#include "stm32f103c8.h"
#include <stdint.h>
#include "printf.h"
#include "usb.h"
//#include "main.h"

//XXX TODO check support for long integers, 64bit arithmetic, etc..

volatile char led = 0;

#define TPIU_DEVID		(*(uint32_t *)(0xE0040FC8))
#define DEM_CR				(*(uint32_t *)(0xE000EDFC))
#define ITM_LOCK			(*(uint32_t *)(0xE0000FB0))
#define ITM_TCR				(*(uint32_t *)(0xE0000E80))
#define ITM_TPR				(*(uint32_t *)(0xE0000E40))
#define ITM_TER				(*(uint32_t *)(0xE0000E00))
#define ITM_PORT0			(*(volatile uint32_t *)(0xE0000000))
#define ITM_PORT0_8		(*(volatile uint8_t *)(0xE0000000))

void enable_ITM(void){
	DEM_CR = 1<<24; //TRCENA   - DEMCR -> 0xE000EDFC
	DBGMCU->CR = DBGMCU_CR_TRACE_IOEN;
	ITM_LOCK = 0xC5ACCE55; //unlock write access
	ITM_TCR = 0x00010005; //trace and control register
	ITM_TER = 1;
	ITM_TPR = 1;
}

void _putchar(char c){ //used by printf lib.
	while(ITM_PORT0 == 0);
	ITM_PORT0_8 = c;
}

void delay(int time_us){
	volatile int i=0;
	for(i=0; i < time_us*4 + (time_us>>2); i++);
}

void init_clock(void){
	FLASH->ACR |= FLASH_ACR_LATENCY_2;
	RCC->CFGR = RCC_CFGR_PLLSRC  | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLXTPRE_HSE | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV2 | RCC_CFGR_ADCPRE_DIV6;
	//RCC->CFGR = RCC_CFGR_PLLSRC  | RCC_CFGR_PLLMULL9 | RCC_CFGR_PLLXTPRE_HSE | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_ADCPRE_DIV6;

	RCC->CR |= RCC_CR_HSEON; while(!(RCC->CR & RCC_CR_HSERDY));
	RCC->CR |= RCC_CR_PLLON; while(!(RCC->CR & RCC_CR_PLLRDY));

	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;

	RCC->APB2ENR = RCC_APB2ENR_IOPCEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_ADC1EN | RCC_APB2ENR_USART1EN | RCC_APB2ENR_AFIOEN;
	RCC->APB1ENR = RCC_APB1ENR_TIM2EN | RCC_APB1ENR_I2C1EN | RCC_APB1ENR_SPI2EN;
	RCC->AHBENR = RCC_AHBENR_DMA1EN | RCC_AHBENR_CRCEN;
}

void init_usart(void){
	GPIOA->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1; //alternate function push pull
	GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
	USART1->CR1 |= USART_CR1_UE;
	//USART1->BRR = 3750; //USART1->BRR = 312; //625/2; //(39 << 4) + 1; //115200
	USART1->BRR = 312; //625/2; //(39 << 4) + 1; //115200
	USART1->BRR = (1 << 4); //2250000
	USART1->CR1 |= USART_CR1_TE;
}

void send_uart(char c){
	USART1->DR = c;
	while(!(USART1->SR & USART_SR_TXE));
	//while(!(USART1->SR & USART_SR_TC));
	//printf("%c",c);
}

void uart_print_number(int number){
	send_uart('0' + (number/1000)); number %= 1000;
	send_uart('0' + (number/100)); number %= 100;
	send_uart('0' + (number/10)); number %= 10;
	send_uart('0' + (number));
}

int main(void){
	init_clock();
	enable_ITM();
	GPIOC->CRH = GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0; //pin 13 output @ 50MHz
	GPIOC->ODR |= 1<<13; //led off     BIT_BAND_PERIPH(&GPIOC->ODR,13) = 1; //led off
	TIM2->CR1 |= TIM_CR1_CEN;
	init_usart();

	printf("started!\n");
	send_uart('a');
	send_uart('b');
	send_uart('c');
	send_uart('d');
	send_uart('e');
	usb_init();
	printf("1!\n");
	//USB->CNTR |= USB_CNTR_FRES;
  //USB->CNTR &= ~USB_CNTR_FRES;
	printf("finished!\n");
	__enable_irq(); //enabled by defualt
	while(1);
	//while(1) __WFI(); //doesnt work when debugging
}
