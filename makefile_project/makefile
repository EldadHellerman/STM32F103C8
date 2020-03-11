CC = arm-none-eabi-gcc
CFLAGS = -mthumb -mcpu=cortex-m3 -march=armv7-m -Wall -std=c99 -Os
AS = arm-none-eabi-as
ASFLAGS = -mthumb -mcpu=cortex-m3 -march=armv7-m

TARGET_PATH = Debug\final.o

.PHONY: all clean

%.o: %.c
		@$(CC) $< -o Debug\$@  -c $(CFLAGS)
		@echo compiling $<

%.o: %.s
		@$(AS) $< -o Debug\$@ $(ASFLAGS)
		@echo compiling $<

all: main.o c_runtime.o vector_table.o printf.o
		@echo linking
#		@arm-none-eabi-ld Debug\main.o Debug\c_runtime.o Debug\vector_table.o Debug\printf.o -T STM32F103C8_linker.ld -o Debug\final.o
		@arm-none-eabi-gcc -nostartfiles Debug\main.o Debug\c_runtime.o Debug\vector_table.o Debug\printf.o -T STM32F103C8_linker.ld -o Debug\final.o
		@echo converting to binary and dissassembling
		@arm-none-eabi-objcopy -O binary Debug\final.o Debug\final.bin
#		arm-none-eabi-objdump Debug\final.bin -marm -D -bbinary -Mforce-thumb
#		arm-none-eabi-objdump -marm -D Debug\final.o
#		arm-none-eabi-objdump -marm -D Debug\final.o > Debug\final.lss   #doesn't work! created a shell script instead:
		@disassemble.cmd
		@echo done!
		@echo

clean:
	del Debug

upload:
	@ST-LINK_CLI -c SWD FREQ=4000 UR -P Debug\final.bin 0x08000000 -V after_programming -Rst
