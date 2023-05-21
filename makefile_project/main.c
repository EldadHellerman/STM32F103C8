#include "stm32f103c8.h"
#include <stdint.h>
#include "printf.h"
#include "sd.h"
#include "usb.h"
#include "usb_hid.h"
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

void print(char *s){
	while(*s != 0){
		while(ITM_PORT0 == 0);
		ITM_PORT0_8 = *s++;
	}
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

void init_tim2(void){
	TIM2->ARR = 10000/60 - 1; //TIM2->ARR = 10000 - 1; //Auto reload value
	TIM2->PSC = 7200; //prescaler
  TIM2->DIER |= 1; //enable interrupts
	NVIC->ISER0 |= 0x10000000; //enable timer2 interrupt
}

void init_usart(void){
	GPIOA->CRH |= GPIO_CRH_MODE9_0 | GPIO_CRH_MODE9_1 | GPIO_CRH_CNF9_1; //alternate function push pull
	GPIOA->CRH &= ~GPIO_CRH_CNF9_0;
	USART1->CR1 |= USART_CR1_UE;
	//USART1->BRR = 3750; //USART1->BRR = 312; //625/2; //(39 << 4) + 1; //115200
	USART1->BRR = 312; //625/2; //(39 << 4) + 1; //115200
	USART1->CR1 |= USART_CR1_TE;
}

void init_adc(void){
	/*comparing adc:							atmega328p					STM32F103C8
	max clock speed: 							200kHz							14MHz
	cycles per conversion:				13									14-252 (dependes on sampling time)
	tiem per conversion:					65-250uS						(1||1.17)-18uS (dependes on clock and sampling time)
	max sampling rate: 						15.5kHz							1MHz
	sampling capacitor:						14pF								8pF

	STM32F103C8 - ADC ch16 is temp sensor, ch17 is vref of 1.2V
	*/
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
	delay(1); //let ADC stablize (takes 1 uS)
	ADC1->CR2 |= ADC_CR2_CAL;		//calibrate the ADC
	ADC1->CR1 |= ADC_CR1_EOCIE;
	ADC1->SMPR1 = (ADC_SMPR1_SMP17_0 | ADC_SMPR1_SMP17_1 | ADC_SMPR1_SMP17_2) | (ADC_SMPR1_SMP16_0 | ADC_SMPR1_SMP16_1); //ch16 & ch17 28.5 cycles sampling time
	ADC1->SMPR2 = 0b000; //0b111; //ch0 239.5 cycles sampling time
	ADC1->SQR1 = ADC_SQR1_L_0; //2 conversions //ADC1->SQR3 = 0;	//fill everyhting with ch0
	ADC1->SQR3 = 0;
	GPIOA->CRL &= ~GPIO_CRL_CNF0; //analog mode
	//GPIOA->CRL = GPIO_CRL_MODE0_1; //GPIOA->ODR = 1;
	//NVIC->ISER0 |= 0x40000; //enable adc1_2 interrupt
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

void blink_led(void){
	led = !led;
	//BIT_BAND_PERIPH(&GPIOC->ODR ,13) = led;
	if(led) GPIOC->BSRR = GPIO_BSRR_BS13; else GPIOC->BRR = GPIO_BRR_BR13;
	//if(led) GPIOC->ODR &= ~(1<<13); else GPIOC->ODR |= 1<<13;
	//printf("LED turned %s\n",(led==1) ? "On" : "Off");
}

int get_adc(void){
	ADC1->CR2 |= ADC_CR2_ADON;
	while((ADC1->SR & ADC_SR_EOC) == 0);
	return(ADC1->DR);
}

int pre = 0;
void _IRQ28_TIM2(void){
	TIM2->SR &= ~1; //clear interrupt flag
	//blink_led();

	//TODO use continous mode to get multiple ADC readings fast
	GPIOA->ODR = 1;
	GPIOA->CRL = 0b0011; delay(5); GPIOA->CRL = 0;
	/*ADC1->SQR3 = 17;
	int reading = get_adc();
	uart_print_number(reading);
	send_uart(',');*/
	ADC1->SQR1 = 0; //1 conversion    ADC_SQR1_L_3 | ADC_SQR1_L_2 | ADC_SQR1_L_1 | ADC_SQR1_L_0; //8 conversions
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 1; int reading_0 = get_adc();
	//ADC1->SQR3 = 1; int reading_1 = get_adc();
	uart_print_number(reading_0);

	#define AMOUNTOFREADINGS 20
	int readings[AMOUNTOFREADINGS];
	for(int t=0; t<AMOUNTOFREADINGS; t++){
		GPIOA->CRL = 0b0011; delay(5); GPIOA->CRL = 0;
		readings[t] = get_adc();
	}
	//for(int t=0; t<5; t++){ send_uart(','); uart_print_number(readings[t]);}
	int avg=0;
	for(int t=0; t<AMOUNTOFREADINGS; t++) avg += readings[t];
	avg = avg/AMOUNTOFREADINGS;
	send_uart(','); uart_print_number(avg);
	printf("%d\n",avg);
	if(avg-pre > 15){
		printf("press detected!\n");
		blink_led();
	}
	pre = avg;
	GPIOA->ODR = 0;


	GPIOA->CRL = 0b0011; delay(10); GPIOA->CRL = 0;
	get_adc();
	//send_uart(','); uart_print_number(reading_1);
	send_uart('\n');
}

void _IRQ18_ADC1_2(void){
	int r = ADC1->DR;
	printf("interrupt r: %d\n",r);
	delay(50000);
	ADC1->CR2 |= ADC_CR2_ADON; //start another conversion
}

void test_adc(void){
	//https://www.st.com/resource/en/application_note/dm00445657-getting-started-with-touch-sensing-control-on-stm32-microcontrollers-stmicroelectronics.pdf
	//http://stm32duinoforum.com/forum/viewtopic_f_15_t_2418.html
	//https://github.com/arpruss/ADCTouchSensor
	ADC1->CR2 = ADC_CR2_ADON | ADC_CR2_TSVREFE;
	delay(1); //let ADC stablize (takes 1 uS)
	ADC1->CR2 |= ADC_CR2_CAL;		//calibrate the ADC
	//ADC1->CR1 |= ADC_CR1_EOCIE;
	ADC1->SMPR1 = (ADC_SMPR1_SMP17_0 | ADC_SMPR1_SMP17_1 | ADC_SMPR1_SMP17_2) | (ADC_SMPR1_SMP16_0 | ADC_SMPR1_SMP16_1); //ch16 & ch17 28.5 cycles sampling time
	ADC1->SMPR2 = 0b000; //0b111; //ch0 239.5 cycles sampling time

	//GPIOA->CRL &= ~GPIO_CRL_CNF0; //analog mode

	int reading1, reading2;
	for(int i=0; i<20; i++){
		ADC1->SQR3 = 17;
		ADC1->CR2 |= ADC_CR2_ADON;
		while((ADC1->SR & ADC_SR_EOC) == 0);
		reading1 = ADC1->DR;

		ADC1->SQR3 = 16;
		ADC1->CR2 |= ADC_CR2_ADON;
		while((ADC1->SR & ADC_SR_EOC) == 0);
		reading2 = ADC1->DR;

		printf("1 - adc is: 1-%d, 2-%d\n",reading1, reading2);
		delay(50000);
	}

	//scan mode:
	volatile uint32_t readings[2];
	DMA->channel[0].CCR = DMA_CCR_PL_VERY_HIGH | DMA_CCR_MSIZE_1 | DMA_CCR_PSIZE_0 | DMA_CCR_MINC | DMA_CCR_CIRC;
	DMA->channel[0].CNDTR = 2;
	DMA->channel[0].CPAR = (int)&ADC1->DR;
	DMA->channel[0].CMAR = (int)readings; //set dst memory address
	DMA->channel[0].CCR |= DMA_CCR_EN;
	ADC1->CR1 |= ADC_CR1_SCAN;
	ADC1->CR2 |= ADC_CR2_DMA;
	for(int i=0; i<20; i++){
		ADC1->SQR1 = ADC_SQR1_L_0;
		ADC1->SQR3 = (16<<5) | 17;
		ADC1->CR2 |= ADC_CR2_ADON;
		delay(50000);
		printf("2 - adc is: 1-%d, 2-%d\n",readings[0], readings[1]);
	}
}

void write_command(char command){
	printf("writing command: %X\n",command);
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = (0x3C << 1) | 0; //address is 0x3C
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	char t = I2C1->SR2;
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = 0b10000000;
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = command;
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	I2C1->CR1 |= I2C_CR1_STOP;
	while(I2C1->CR1 & I2C_CR1_STOP);
}

void write_data(char data){
	printf("writing data: %X\n",data);
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));

	I2C1->DR = (0x3C << 1) | 0; //address is 0x3C
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	char t = I2C1->SR2;

	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = 0b11000000;

	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	I2C1->CR1 |= I2C_CR1_STOP;
	while(I2C1->CR1 & I2C_CR1_STOP);
}

void write_data_con_start(){
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = (0x3C << 1) | 0; //address is 0x3C
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	char t = I2C1->SR2;
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = 0b01000000;
}

void write_data_con_stop(){
	while(!(I2C1->SR1 & I2C_SR1_BTF));
	I2C1->CR1 |= I2C_CR1_STOP;
	while(I2C1->CR1 & I2C_CR1_STOP);
}

void write_data_con(char data){
	while(!(I2C1->SR1 & I2C_SR1_TXE));
	I2C1->DR = data;
}

void write_data_dma_start(){
	I2C1->CR1 |= I2C_CR1_START;
	while(!(I2C1->SR1 & I2C_SR1_SB));
	I2C1->DR = (0x3C << 1) | 0; //address is 0x3C
	while(!(I2C1->SR1 & I2C_SR1_ADDR));
	char t = I2C1->SR2;
}

void test_oled(void){
	//init i2c:
	GPIOB->CRL = 0xff000000;
	//I2C1->CCR = 160; //arbitrary number since i dont understand how it affects the clock speed
	I2C1->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 2;
	I2C1->CR2 = 50;
	I2C1->CR1 = I2C_CR1_PE;
	delay(10000);
	write_command(0x8D);
	write_command(0x14);
	write_command(0xAF); //turn on display
	write_command(0x81); write_command(255);
	//TODO try using DMA with I2C, page 767 on TM
	for(long i=0; i<2; i++){
		write_command(0xA5); //force all pixels to be on regardless of GRAM
		delay(1000000);
		write_command(0xA4);
		delay(1000000);
	}
	write_command(0x20); //horizontal addressing
	write_command(0x21); write_command(0); write_command(127);
	write_command(0x22); write_command(0); write_command(7);
	write_data_con_start();
	for(int l=0; l<30; l++){
		for(long i=0; i<128*8; i++) write_data_con(0x00);
		for(long i=0; i<128*8; i++) write_data_con(0xff);
	}
	write_data_con_stop();
	delay(2000000);
	write_data_con_start();
	for(int i=0; i<128*8; i++) write_data_con(0x00);
	write_data_con_stop();
}

void init_oled(void){
	write_command(0xAE);
	write_command(0x81); write_command(255); //set contrast to 255
	write_command(0x2E); //stop scrolling
	write_command(0x40); //set display start line
	write_command(0xA8); write_command(63); //set mux ratio
	write_command(0xDA); write_command(0x12); //configure COM pins
	write_command(0xD5); write_command(0x80); //set clock
	write_command(0xD9); write_command(0x22); //set precharge ratio
	write_command(0xDB); write_command(0x20); //set VCOMh Deselect level
	write_command(0xD3); write_command(0x00); //vertical offset
	write_command(0xA0); //vertical mapping 0-127 -> left to right
	write_command(0xC8); //vertical mapping 0-63 -> up to down
	write_command(0xA6); //display non-inverse
	write_command(0xA4); //turn off 'entire display on'
	write_command(0x8D); write_command(0x14); //turn on charge pump
	write_command(0xAF); //turn on display
	write_command(0x20); write_command(0x00); //horizontal addressing
	write_command(0x21); write_command(0); write_command(127);
	write_command(0x22); write_command(0); write_command(7);
}
/*
#include "icons.h"
void test_oled_dma(void){
	GPIOB->CRL = 0xff000000;
	I2C1->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 2;
	I2C1->CR2 = 50 | I2C_CR2_DMAEN;
	DMA->channel[5].CPAR = (int)&I2C1->DR;
	DMA->channel[5].CMAR = (int)oled_buffer;
	DMA->channel[5].CNDTR = 8*128 + 1;
	DMA->channel[5].CCR = DMA_CCR_PL_VERY_HIGH | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC;
	I2C1->CR1 = I2C_CR1_PE;
	for(int k=1; k<8*128 + 1; k++) oled_buffer[k] = 0xf0;
	oled_buffer[0] = 0b01000000;
	delay(10000);
	init_oled();
	for(long i=0; i<2; i++){
		write_command(0xA5); //force all pixels to be on regardless of GRAM
		delay(1000000);
		write_command(0xA4);
		delay(1000000);
	}
	write_data_dma_start();
	DMA->channel[5].CCR |= DMA_CCR_EN;
	while(!(DMA->ISR & DMA_ISR_TCIF6));
	DMA->channel[5].CCR &= ~DMA_CCR_EN;
	DMA->IFCR = DMA_IFCR_CTCIF6;
	write_data_con_stop();
	delay(1000000);

	for(int k=1; k<8*128 + 1; k++) oled_buffer[k] = 0x0f;
	write_data_dma_start();
	DMA->channel[5].CCR |= DMA_CCR_EN;
	while(!(DMA->ISR & DMA_ISR_TCIF6));
	DMA->channel[5].CCR &= ~DMA_CCR_EN;
	DMA->IFCR = DMA_IFCR_CTCIF6;
	write_data_con_stop();
	delay(1000000);

	DMA->channel[5].CMAR = (int)icon_1;
	write_data_dma_start();
	DMA->channel[5].CCR |= DMA_CCR_EN;
	while(!(DMA->ISR & DMA_ISR_TCIF6));
	DMA->channel[5].CCR &= ~DMA_CCR_EN;
	DMA->IFCR = DMA_IFCR_CTCIF6;
	write_data_con_stop();
	delay(2000000);

	while(1){
		for(int f=0; f<100; f++){
			DMA->channel[5].CMAR = (int)mov[f];
			write_data_dma_start();
			DMA->channel[5].CCR |= DMA_CCR_EN;
			while(!(DMA->ISR & DMA_ISR_TCIF6));
			DMA->channel[5].CCR &= ~DMA_CCR_EN;
			DMA->IFCR = DMA_IFCR_CTCIF6;
			write_data_con_stop();
			delay(17000);
		}
	}
}
*/

void oled_display(char *buffer){
	DMA->channel[5].CMAR = (int)buffer;
	write_data_dma_start();
	DMA->channel[5].CCR |= DMA_CCR_EN;
	while(!(DMA->ISR & DMA_ISR_TCIF6));
	DMA->channel[5].CCR &= ~DMA_CCR_EN;
	DMA->IFCR = DMA_IFCR_CTCIF6;
	write_data_con_stop();
}

void test_sd(void){
	SD_CARD card;
	sd_init(&card);
	if(!card.error){
		//init_fast_spi();
		printf("successful!!!   ");
		if(card.status & SD_CARD_HIGH_CAPACITY) printf("HC");
		if(card.status & SD_CARD_VERSION_2) printf("SD version >= 2.00!\n"); else printf("sd version 1.x!\n");

		GPIOB->CRL = 0xff000000;
		I2C1->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 2;
		I2C1->CR2 = 50 | I2C_CR2_DMAEN;
		DMA->channel[5].CPAR = (int)&I2C1->DR;
		DMA->channel[5].CNDTR = 8*128 + 1;
		DMA->channel[5].CCR = DMA_CCR_PL_VERY_HIGH | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC;
		I2C1->CR1 = I2C_CR1_PE;
		init_oled();

		for(int i=0; i<1000; i++){
			byte buffer[8*128 + 1];
			buffer[0] = 0x40;
			read_bytes_from_block(&card, 0, buffer+1, 0, 512);
			read_bytes_from_block(&card, 1, buffer+1+512, 0, 512);
			oled_display(buffer);
			read_bytes_from_block(&card, 2, buffer+1, 0, 512);
			read_bytes_from_block(&card, 3, buffer+1+512, 0, 512);
			oled_display(buffer);
			read_bytes_from_block(&card, 4, buffer+1, 0, 512);
			read_bytes_from_block(&card, 5, buffer+1+512, 0, 512);
			oled_display(buffer);
		}
		/*
		printf("block read:\n");
		for(int i=0; i<32; i++){
			printf("%.2X:\t", i*16);
			for(int k=0; k<16; k++) printf("0x%.2X,",buffer[i*16 + k]);
			print("\t|\t");
			for(int k=0; k<16; k++) printf("%c", (buffer[i*16 + k] != (byte)0) ? buffer[i*16 + k] : " ");
			print("\n");
			delay(100000);
		}*/
		printf("done printing contents of sd card\n");
	}else{
		printf("error... number %d, error %.2X\n",card.errornum,card.error);
	}
}

void oled_play_movie_from_sd(void){
	SD_CARD card;
	sd_init(&card);
	if(!card.error){
		//init_fast_spi();
		printf("successful!!!   ");
		if(card.status & SD_CARD_HIGH_CAPACITY) printf("HC");
		if(card.status & SD_CARD_VERSION_2) printf("SD version >= 2.00!\n"); else printf("sd version 1.x!\n");

		GPIOB->CRL = 0xff000000;
		I2C1->CCR = I2C_CCR_FS | I2C_CCR_DUTY | 2;
		I2C1->CR2 = 50 | I2C_CR2_DMAEN;
		DMA->channel[5].CPAR = (int)&I2C1->DR;
		DMA->channel[5].CNDTR = 8*128 + 1;
		DMA->channel[5].CCR = DMA_CCR_PL_VERY_HIGH | DMA_CCR_MINC | DMA_CCR_DIR | DMA_CCR_CIRC;
		I2C1->CR1 = I2C_CR1_PE;
		init_oled();
		byte buffer[8*128 + 1];
		buffer[0] = 0x40;
		DMA->channel[5].CMAR = (int)buffer;
		for(int pic=0; pic<1000; pic++){
			//read_block(&card, pic*2, buffer+1);
			//read_block(&card, pic*2 + 1, buffer+1+512);
			read_block_dma(&card, pic*2, buffer+1);
			read_block_dma(&card, pic*2 + 1, buffer+1+512);
			write_data_dma_start();
			DMA->channel[5].CCR |= DMA_CCR_EN;
			while(!(DMA->ISR & DMA_ISR_TCIF6));
			DMA->channel[5].CCR &= ~DMA_CCR_EN;
			DMA->IFCR = DMA_IFCR_CTCIF6;
			write_data_con_stop();
			//delay(500000);
		}
		printf("done printing contents of sd card\n");
	}else{
		printf("error... number %d, error %.2X\n",card.errornum,card.error);
	}
}

int main(void){
	init_clock();
	enable_ITM();
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
	GPIOC->CRH = GPIO_CRH_MODE13_1 | GPIO_CRH_MODE13_0; //pin 13 output @ 50MHz
	GPIOC->ODR |= 1<<13; //led off     BIT_BAND_PERIPH(&GPIOC->ODR,13) = 1; //led off
	TIM2->CR1 |= TIM_CR1_CEN;
	init_usart();
	//init_tim2();
	printf("started!\n");
	//while(1);
	//init_adc(); test_adc();
	//test_sd();
	//test_oled_dma();
	//oled_play_movie_from_sd();
	usb_init();
	printf("1!\n");
	hid_init();
	printf("2!\n");
	usb_print_status();
	printf("3!\n");
	USB->CNTR |= USB_CNTR_FRES;
  USB->CNTR &= ~USB_CNTR_FRES;
	printf("finished!\n");

	__enable_irq(); //enabled by defualt
	while(1);
	/*ADC1->SQR1 = 0;
	//ADC1->SQR1 = ADC_SQR1_L_0; //2 conversion
	ADC1->SQR2 = 0;
	ADC1->SQR3 = 1;
	GPIOA->CRL = 0b10; //pull up
	int reading_p = 0;
	int threshhold = 1130;
	while(1){
		GPIOA->CRL = 0b1000 << 4; delay(20000); GPIOA->CRL = 0;
		int reading = get_adc();
		if((reading < threshhold) & (reading_p >= threshhold)) blink_led();
		reading_p = reading;
		uart_print_number(reading);
		send_uart(',');
		uart_print_number(threshhold);
		send_uart('\n');
		//printf("%d\n",get_adc());
	}*/
	/*while(1){
		ADC1->SQR3 = 17;
		int reading = get_adc();
		ADC1->SQR3 = 0;
		reading = get_adc();
		printf("r1: %d\n", reading);
		uart_print_number(reading); //send_uart(get_adc() & 0xff);
		send_uart('\n');
		delay((1000000/50) + 730);
		//GPIOA->ODR = 1; delay(20000);
		//printf("r2: %d\n", get_adc());
		//GPIOA->ODR = 0; delay(20000);
	}*/
	//while(1) __WFI(); //doesnt work when debugging
}


const char string[] = "hello, this is a string which the DMA shold copy to a different location in memory";

void dma_init(void){
	DMA->channel[1].CCR = DMA_CCR_MEM2MEM | DMA_CCR_PL_VERY_HIGH | DMA_CCR_PINC | DMA_CCR_MINC;
	DMA->channel[1].CNDTR = 83; //set number of bytes
	DMA->channel[1].CPAR = (int)string; //set src memory address
	DMA->channel[1].CMAR = SRAM_BASE + 0x100; //set dst memory address
}

void dma_start(void){
	DMA->channel[1].CCR |= DMA_CCR_EN; //activate the channel
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
