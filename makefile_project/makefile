ODIR = ./Build
_OBJ = main.o c_runtime.o vector_table.o printf.o

CC = arm-none-eabi-gcc
CFLAGS = -mthumb -mcpu=cortex-m3 -march=armv7-m -Wall -std=c99 -Os -MMD -MP
AS = arm-none-eabi-as
ASFLAGS = -mthumb -mcpu=cortex-m3 -march=armv7-m
OBJ := $(patsubst %,$(ODIR)/%,$(_OBJ))
DEPS := $(OBJ:.o=.d)

#auto generating objects from each source code file:
#SRCS := $(shell find $(SRC_DIRS) -name *.c -or -name *.s) #finds all .c|.s files and saves their names
#OBJS := $(SRCS:%=$(ODIR)/%.o) #prepends ODIR and appes .o

#auto generate GCC include flags for all directories in source folder
#INC_DIRS := $(shell find $(SRC_DIRS) -type d) #find all directories in a source directory folder
# INC_FLAGS := $(addprefix -I,$(INC_DIRS)) #and prepend a -I flag

.PHONY: all clean

all: $(OBJ)
		@echo linking
#		@arm-none-eabi-ld Debug\main.o Debug\c_runtime.o Debug\vector_table.o Debug\printf.o -T STM32F103C8_linker.ld -o Debug\final.o
		@arm-none-eabi-gcc -nostartfiles $(OBJ) -T STM32F103C8_linker.ld -o $(ODIR)\final.o
		@echo converting to binary and dissassembling
		@arm-none-eabi-objcopy -O binary $(ODIR)\final.o $(ODIR)\final.bin
#		arm-none-eabi-objdump Debug\final.bin -marm -D -bbinary -Mforce-thumb
#		arm-none-eabi-objdump -marm -D Debug\final.o
#		arm-none-eabi-objdump -marm -D Debug\final.o > Debug\final.lss   #doesn't work! created a shell script instead:
		@disassemble.cmd
		@echo done!
		@echo

-include $(DEPS)

$(ODIR)/%.o: %.c
	@$(CC) $< -o $@  -c $(CFLAGS)
	@echo compiling $<

$(ODIR)/%.o: %.s
	@$(AS) $< -o $@ $(ASFLAGS)
	@echo compiling $<

	# rm "$(ODIR)/*"
	# rm -f "Build/*.*"
clean:
	rm -r "Build"
# del Build

upload:
	@ST-LINK_CLI -c SWD FREQ=4000 UR -P $(ODIR)\final.bin 0x08000000 -V after_programming -Rst
