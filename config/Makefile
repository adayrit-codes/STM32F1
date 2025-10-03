all: main.o firmware.elf firmware.bin flash

main.o: main.c
	arm-none-eabi-gcc -mcpu=cortex-m3 -c main.c -I. "../config"

firmware.elf: main.o
	arm-none-eabi-gcc -T linker.ld -nostdlib main.o -o firmware.elf 

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary firmware.elf firmware.bin

flash: firmware.bin
	st-flash write firmware.bin 0x08000000

clean:
	rm -f *.o *.elf *.bin
.PHONY: all clean flash