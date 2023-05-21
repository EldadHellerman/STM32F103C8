#include "stm32f103c8.h"
#include "sd.h"

#define CS_LOW()	GPIOB->BRR = 1<<12;
#define CS_HIGH()	GPIOB->BSRR = 1<<12;

static byte spi_transfer(byte data);
static byte send_command(byte command, uint32_t arguments, byte crc);

void spi_init(){ //MSB first, 100-400 kHz
  //NSS B12 as GPIO-PP, SCK -> B13 as AF-PP, MISO -> B14 as floating input, MOSI -> B15 as AF-PP
  GPIOB->CRH = 0xB4B34444;
  SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR; //can probably increase SPI speed from 250kHz
  //SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_0 | SPI_CR1_MSTR;
  SPI2->CR1 |= SPI_CR1_SPE;
	CS_HIGH();
}

byte spi_transfer(byte data){
	SPI2->DR = data;
  //while(!(SPI2->SR & (1<<SPI_SR_TXE)));
	while(!(SPI2->SR & (1<<SPI_SR_RXNE)));
	return(SPI2->DR);
}

byte sd_init(SD_CARD *card){
	//steps:
	//	1.dummy clock for 80 clocks
	//	2.CMD0
	//	3.CMD8(0x1AA)
	//	if sd version 1.x:
	//		4.ACMD41
	//	else, sd ver 2.0 and above:
	//		4.ACMD41(1<<30)
	//		5.CMD58 to determine if HC or not.
  spi_init();
	card->status = 0;
	card->error = 1;
	do{
		for(byte i=0; i<10; i++) spi_transfer(0xFF); //dummy clock > 74 (80)
		CS_LOW(); for(volatile int t=0; t<1000000; t++); // _delay_ms(10);
		if(send_command(CMD0,0,CMD0_CRC) != 0x01){ card->errornum = 1; break;}
		card->error = send_command(CMD8,0x1AA,SD_CRC(0x43));
		byte buffer[4] = {0};
		if(card->error == 0x01){
			card->status |= SD_CARD_VERSION_2;
			spi_transfer(0xff); spi_transfer(0xff); //get rid of first 2 bytes
			buffer[0] = spi_transfer(0xff); buffer[1] = spi_transfer(0xff);
			//printf("buffer[0]: %X, buffer[1]: %X\n",buffer[0],buffer[1]); //TODO check what is 'check pattern'
			if(buffer[0] != 0x01 || buffer[1] != 0xAA){ card->errornum=2; break;}
		}
		byte attempts = 250;
		while(attempts-- > 0){
			if(send_command(CMD55,0,0) != 1){ card->errornum=3; break;}
			card->error = send_command(CMD41, 0x40000000, 0); //(card->status & SD_CARD_VERSION_2) ? 0x40000000 : 0
			if((card->error & 0xFE) || (card->error == 0)) break; //if error or initiated
		}
		if(card->error != 0){ card->errornum=4; break;}
		if(send_command(CMD16,0x00000200,0)){ card->errornum=5; break;}
		if(card->status & SD_CARD_VERSION_2){
			if(send_command(CMD58,0,0)){ card->errornum=6; break;}
			for(byte i=0; i<4; i++) buffer[i] = spi_transfer(0xff);
			if(buffer[0] & 0x40) card->status |= SD_CARD_HIGH_CAPACITY;
			//printf("OCR is %X,%X,%X,%X\n",buffer[0],buffer[1],buffer[2],buffer[3]);
		}
		card->error = 0;
	}while(0);
	CS_HIGH();
	spi_transfer(0xff); //clocks needed to let sd card finish operation (according to SanDisk docs)
	return(card->error);
}

byte send_command(byte command, uint32_t argument, byte crc){
	spi_transfer(0xff); //to provide 8 clocks that according to timings charts in SanDisk SD Docs is optional, and are used by arduino.
	//excluding the above 8 clock makes it fail with newer SanDisk SD cards
	spi_transfer(command);
	spi_transfer((argument>>24) & 0xff);
	spi_transfer((argument>>16) & 0xff);
	spi_transfer((argument>>8) & 0xff);
	spi_transfer((argument>>0) & 0xff);
	spi_transfer(crc);
	byte data, attempts=16; //SanDisk Docs says 8 attempts should be the max
	do data = spi_transfer(0xFF); while(data&0x80 && (attempts-- != 0)); //check multiple times for response
	return(data);
}

byte read_block_dma(SD_CARD *card, uint32_t address, byte *buffer){
  SPI2->CR1 &= ~SPI_CR1_SPE;
  SPI2->CR1 = SPI_CR1_SSM | SPI_CR1_SSI | SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_MSTR;
	byte error;
	if(!(card->status & SD_CARD_HIGH_CAPACITY)) address <<= 9;
	CS_LOW();
	do{
		if((error = send_command(CMD17,address,0))) break;
		byte attempts = 250;
		do error = spi_transfer(0xff); while(error != 0xFE && (error & 0xF0) && attempts-- > 0);
		if(!(error & 0xF0)) break; //if there was an error in loop

    DMA->channel[3].CPAR = (int)&SPI2->DR;
    DMA->channel[3].CMAR = (int)buffer;
		DMA->channel[3].CNDTR = 512;
		DMA->channel[3].CCR = DMA_CCR_PL_VERY_HIGH | DMA_CCR_MINC | DMA_CCR_CIRC;
    DMA->channel[3].CCR |= DMA_CCR_EN;

    byte dummy = 0xff;
    DMA->channel[4].CPAR = (int)&SPI2->DR;
    DMA->channel[4].CMAR = (int)&dummy;
		DMA->channel[4].CNDTR = 512;
		DMA->channel[4].CCR = DMA_CCR_PL_HIGH | DMA_CCR_DIR | DMA_CCR_CIRC;
    DMA->channel[4].CCR |= DMA_CCR_EN;
    SPI2->CR2 |= SPI_CR2_RXDMAEN;
    SPI2->CR1 |= SPI_CR1_SPE;
    SPI2->CR2 |= SPI_CR2_TXDMAEN;

    while(!(DMA->ISR & DMA_ISR_TCIF5));
    //while(!(SPI2->SR & (1<<SPI_SR_TXE)));
    //while(SPI2->SR & (1<<SPI_SR_BSY));
		spi_transfer(0xff); spi_transfer(0xff); //for 16 bit crc
	}while(0);
	CS_HIGH();
	spi_transfer(0xff); //padding of additional 8 clocks
	return(error);
}

byte read_block(SD_CARD *card, uint32_t address, byte *buffer){
	byte error;
	if(!(card->status & SD_CARD_HIGH_CAPACITY)) address <<= 9;
	CS_LOW();
	do{
		if((error = send_command(CMD17,address,0))) break;
		byte attempts = 250;
		do error = spi_transfer(0xff); while(error != 0xFE && (error & 0xF0) && attempts-- > 0);
		if(!(error & 0xF0)) break; //if there was an error in loop
    for(int i=0; i<512; i++) buffer[i] = spi_transfer(0xff);
		spi_transfer(0xff); spi_transfer(0xff); //for 16 bit crc
	}while(0);
	CS_HIGH();
	spi_transfer(0xff); //padding of additional 8 clocks
	return(error);
}

byte read_bytes_from_block(SD_CARD *card, uint32_t address, byte *buffer, int start, int end){
	byte error;
	if(!(card->status & SD_CARD_HIGH_CAPACITY)) address <<= 9;
	CS_LOW();
	do{
		if((error = send_command(CMD17,address,0))) break;
		byte attempts = 250;
		do error = spi_transfer(0xff); while(error != 0xFE && (error & 0xF0) && attempts-- > 0);
		if(!(error & 0xF0)) break; //if there was an error in loop
		byte data;
		for(int i=0; i<512; i++){
			data = spi_transfer(0xff);
			if(i>=start && i<end) buffer[i-start] = data;
		}
		spi_transfer(0xff); spi_transfer(0xff); //for 16 bit crc
	}while(0);
	CS_HIGH();
	spi_transfer(0xff); //padding of additional 8 clocks
	return(error);
}
