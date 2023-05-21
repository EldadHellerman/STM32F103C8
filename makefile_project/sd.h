#ifndef SD_H
#define SD_H

#define SD_CMD(x) (0b01000000 | x)
#define SD_CRC(x) (x<<1) | 1

#define CMD0	SD_CMD(0) //software reset (resets the SD card)
#define CMD1	SD_CMD(1) //send operating conditions (activates the card's initialization process)
#define CMD8	SD_CMD(8) //send IF conditions (for SDC V2. check voltage range)
#define CMD9	SD_CMD(9) //send CSD (card specific data)
#define CMD10	SD_CMD(10) //send CID (card identification)
#define CMD12	SD_CMD(12) //stop transmission (during multiple blocks read)
#define CMD16	SD_CMD(16) //set block length (in bytes) - argument is length in bytes
#define CMD17	SD_CMD(17) //read a block - argument is data address
#define CMD18	SD_CMD(18) //read multiple blocks - argument is data address
//#define CMD23	SD_CMD(23) //ACMD23 - set write block erase count (set the number of write blocks to be pre-erased before writing)
#define CMD24	SD_CMD(24) //write a block - argument is data address
#define CMD25	SD_CMD(25) //write multiple blocks - argument is data address
#define CMD41	SD_CMD(41) //ACMD41 - SD send operating conditions (activates the card's initialization process)
#define CMD55	SD_CMD(55) //APP command next (notify card that next command is an app specific command)
#define CMD58	SD_CMD(58) //read OCR (OCR register)
#define CMD59	SD_CMD(59) //CRC on / off - argument[0] selects on or off

#define CMD0_CRC	SD_CRC(0x4A)
#define APP_CMD		CMD55

typedef uint32_t FAT_entry; //used both as index and value in FATs
typedef uint32_t FAT32_filesize;

typedef FAT_entry directory;
typedef uint8_t byte;

#define SD_CARD_VERSION_2		0x01
#define SD_CARD_HIGH_CAPACITY	0x02

struct SD_CARD{
	byte error;
	byte errornum;
	byte status;
	uint32_t first_partition_LBA;
	uint32_t first_data_cluster_LBA;
	uint16_t reserved_sectors;
	//for the first (valid) partition:
	//uint16_t bytes_per_sector;
	//byte sectors_per_cluster;
	byte number_of_FATS;
	uint32_t fs_size;
	uint32_t fat_size;
	directory root;
};

typedef struct SD_CARD SD_CARD;

byte sd_init(SD_CARD *card);
byte read_block(SD_CARD *card, uint32_t address, byte *buffer);
byte read_block_dma(SD_CARD *card, uint32_t address, byte *buffer);
byte read_bytes_from_block(SD_CARD *card, uint32_t address, byte *buffer, int start, int end);
#endif
