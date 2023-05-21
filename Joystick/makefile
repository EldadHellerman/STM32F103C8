LIB_PATH = E:/hobbys/Programing/ARM/STM32F103C8/lib
ODIR = Build
_OBJ = main.o usb.o printf.o


CC = arm-none-eabi-gcc
CFLAGS = -mthumb -mcpu=cortex-m3 -march=armv7-m -Wall -std=c99 -Os -MMD -MP -I $(LIB_PATH)
AS = arm-none-eabi-as -I $(LIB_PATH)
ASFLAGS = -mthumb -mcpu=cortex-m3 -march=armv7-m
OBJ := $(patsubst %,$(ODIR)/%,$(_OBJ))
DEPS := $(OBJ:.o=.d)

#auto generating objects from each source code file:
#SRCS := $(shell find $(SRC_DIRS) -name *.c -or -name *.s) #finds all .c|.s files and saves their names
#OBJS := $(SRCS:%=$(ODIR)/%.o) #prepends ODIR and appes .o

#auto generate GCC include flags for all directories in source folder
#INC_DIRS := $(shell find $(SRC_DIRS) -type d) #find all directories in a source directory folder
# INC_FLAGS := $(addprefix -I,$(INC_DIRS)) #and prepend a -I flag

.PHONY: first all clean upload

first: all
	@echo done!
	@echo

-include $(DEPS)

all: $(OBJ) upload
		@echo linking
#		@arm-none-eabi-ld Debug\main.o Debug\c_runtime.o Debug\vector_table.o Debug\printf.o -T STM32F103C8_linker.ld -o Debug\final.o
		@arm-none-eabi-gcc -nostartfiles $(OBJ) -o $(ODIR)/final.o $(LIB_PATH)/bin/c_runtime.o $(LIB_PATH)/bin/vector_table.o -T $(LIB_PATH)/STM32F103C8_linker.ld
		@echo converting to binary and dissassembling
		@arm-none-eabi-objcopy -O binary $(ODIR)/final.o $(ODIR)/final.bin
#		arm-none-eabi-objdump Debug\final.bin -marm -D -bbinary -Mforce-thumb #dumping from binary
#		arm-none-eabi-objdump -marm -D Debug\final.o > Debug\final.lss   #doesn't work! created a shell script instead:
		@disassemble.cmd

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
	@ST-LINK_CLI -c SWD FREQ=4000 UR -P $(ODIR)/final.bin 0x08000000 -V after_programming -Rst
