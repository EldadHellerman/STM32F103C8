#include "stm32f103c8.h"
#include <stdint.h>
#include "printf.h"
//#include "main.h"


int main(void);
volatile char led = 0;
const char string[] = "hello, this is a string which the DMA shold copy to a different location in memory";



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

void _putchar(char c){
	while(ITM_PORT0 == 0);
	ITM_PORT0_8 = c;
}

void print(char *s){
	while(*s != 0){
		while(ITM_PORT0 == 0);
		ITM_PORT0_8 = *s++;
	}
}

void _IRQ27_TIM2(void){
	TIM2->SR &= ~1; //clear interrupt flag
	led = !led;
	if(led) GPIOC->ODR &= ~(1<<13); //led on
	else GPIOC->ODR |= 1<<13; //led off
	//printf("LED turned %s\n",(led==1) ? "On" : "Off");
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
	RCC->AHBENR = RCC_AHBENR_DMA1EN | RCC_AHBENR_CRCEN;
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

int crc_32(int input){
	CRC->CR = CRC_CR_RESET;
	__DSB(); //reset should occur before data is written
	CRC->DR = input;
	return(CRC->DR);
}

int main(void){
	init_clock();
	enable_ITM();
	__enable_irq(); //enabled by defualt

	/*
	dma_init();
	stk_start();
	dma_start(); while(!(DMA->ISR & ~DMA_ISR_TCIF1));
	printf("DMA took %d cycles\n", stk_stop());
	printf("org: '%s'\ndst:'%s'\n", string, (char *)(SRAM_BASE + 0x100));
	int r[3];
	r[0] = crc_32(0x12345678); //0xDF8A8A2B
	r[1] = crc_32(0x87654321); //0x99AB297E
	r[2] = crc_32(0xe1dad2e1); //0xBEF7B175
	printf("results are: r1-0x%X, r2-0x%X, r3-0x%X\n", r[0], r[1], r[2]);

	//printf("started, number %d, 0X%p\n",123,0x4567e1d);
	//printf("buffer size: 2^%d\n", (TPIU_DEVID>>6) & 7);
	*/

	GPIOC->CRH = GPIO_CRH_MODE13_1; //pin 13 output @ 2MHz
	GPIOC->ODR |= 1<<13; //led off
	init_tim2();
	TIM2->CR1 |= TIM_CR1_CEN;
	while(1);
	//while(1) __WFI(); //doesnt work when debugging
}
