CFLAGS  ?=  -g\
			-O0\
			-Wall\
			-Wextra\
			-mthumb\
			-mno-thumb-interwork \
			-mcpu=cortex-m4 \
			-mfloat-abi=soft\
			-nostartfiles\
			-specs=nano.specs\
			-fno-rtti\
			-fsingle-precision-constant\
			-fno-exceptions
LDFLAGS ?= -T link.ld -nostartfiles -nostdlib
SOURCES = main.c 

build: firmware.elf

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@

firmware.bin: firmware.elf
	arm-none-eabi-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $< 0x8000000

clean:
	del -rf firmware.*

debug:
	openocd -f "C:/Program Files/OpenOCD/openocd/scripts/board/st_nucleo_l4.cfg
