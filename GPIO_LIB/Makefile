CC=arm-none-eabi-gcc
MACH=cortex-m3
CFLAGS= -c -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11 -o0
CFLAGS_ASM= -S -mcpu=$(MACH) -mthumb -mfloat-abi=soft -std=gnu11
LD_FLAGS= -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=nano.specs -Wl,-Map=final.map -T STM32F103C8_lf.ld
LD_FLAGS_Semi= -mcpu=$(MACH) -mthumb -mfloat-abi=soft --specs=rdimon.specs -Wl,-Map=final_semi.map -T STM32F103C8_lf.ld
all: STM32F103C8_Startup.o Blink.o syscalls.o final.elf
all_semi: STM32F103C8_Startup.o Blink.o  final_semi.elf
obj: Blink.o
Asm: Blink.S
clean:
	del *.map *.o *.s *.elf
Blink.S:Blink.c 
	$(CC) $(CFLAGS_ASM) $^ -o $@
Blink.o:Blink.c 
	$(CC) $(CFLAGS) $^ -o $@
syscalls.o:syscalls.c 
	$(CC) $(CFLAGS) $^ -o $@

STM32F103C8_Startup.o:STM32F103C8_Startup.c 
	$(CC) $(CFLAGS) $^ -o $@

final.elf:STM32F103C8_Startup.o Blink.o syscalls.o
	$(CC) $(LD_FLAGS) $^ -o $@
	
final_semi.elf:STM32F103C8_Startup.o Blink.o
	$(CC) $(LD_FLAGS_Semi) $^ -o $@

load:
	openocd -f interface/stlink-v2.cfg -f board/stm32f103c8_blue_pill.cfg