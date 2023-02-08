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
	rm -rf firmware.*

debug:
	openocd -f "C:/Program Files/OpenOCD/scripts/interface/stlink.cfg" -f board/stm32l4discovery.cfg
