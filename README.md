# Bare-metal tutorial
A bare-metal tutorial for STM32L4R5ZI-P adapted from [here](https://github.com/cpq/bare-metal-programming-guide). NOTE: this tutorial was completed using a Windows machine.

The guide covers the following topics: memory and registers, interrupt vector table, startup code, linker script, build automation using make, GPIO peripheral and LED blinky, SysTick timer, UART peripheral and debug output, printf redirect to UART (IO retargeting), debugging with Segger Ozone and system clock setup. 

Every chapter in this guide comes with a complete source code which gradually progress in functionality and completeness. In this tutorial we'll use the Nucleo-L4R5ZI-P development board, so go ahead and download the "mcu datasheet" and the "board datasheet" for it.

MCU datasheet: [here](https://www.st.com/resource/en/reference_manual/rm0432-stm32l4-series-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
Board datasheet: [here](https://www.st.com/resource/en/user_manual/um2179-stm32-nucleo144-boards-mb1312-stmicroelectronics.pdf)

# Tools setup
To proceed, the following tools are required:
- ARM GCC, https://launchpad.net/gcc-arm-embedded - for compiling and linking
- GNU make, http://www.gnu.org/software/make/ - for build automation
- ST link, https://github.com/stlink-org/stlink - for flashing

## Setup instructions for Windows
- Download and install gcc-arm-none-eabi-10.3-2021.10-win32.exe. Enable "Add path to environment variable" during the installation
- Create c:\tools folder
- Download stlink-1.7.0-x86_64-w64-mingw32.zip and unpack bin/st-flash.exe into c:\tools
- Download make-4.4-without-guile-w32-bin.zip and unpack bin/make.exe into c:\tools
- Add c:\tools to the Path environment variable
- Verify installation:
 - Download and unzip this repository into c:\
 - Start command prompt, and execute the following: 
```
C:\Users\YOURNAME> cd \
C:\> cd bare-metal-programming-guide-main\step-0-minimal
C:\bare-metal-programming-guide-main\step-0-minimal> make
arm-none-eabi-gcc main.c  -W -Wall -Wextra -Werror ...
```

## Setup instructions for Mac
Start a terminal, and execute:
```
$ /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
$ brew install gcc-arm-embedded make stlink
```

## Setup instructions for Linux (Ubuntu)
Start a terminal, and execute:
```
$ sudo apt -y update
$ sudo apt -y install gcc-arm-none-eabi make stlink-tools
```

# Introduction
A microcontroller (uC, or MCU) is a small computer. Typically it has CPU, RAM, flash to store firmware code, and a bunch of pins that stick out. Some pins are used to power the MCU, usually marked as GND (ground) and VCC pins. Other pins are used to communicate with the MCU, by means of high/low voltage applied to those pins. One of the simplest ways of communication is an LED attached to a pin: one LED contact is attached to the ground pin (GND), and another contact is attached to a signal pin via a current-limiting resistor. A firmware code can set high or low voltage on a signal pin, making LED blink.

<img src="images/mcu.svg" height="200" />

## Memory and registers
The 32-bit address space of the MCU is divided by regions. For example, some region of memory is mapped to the internal MCU flash at a specific address. Firmware code instructions are read and executed by reading from that memory region. Another region is RAM, which is also mapped to a specific address. We can read and write any values to the RAM region.

From STM32L4R5 datasheet, we can take a look at section ?? and learn that RAM region starts at address 0x20000000 and has size of 192KB. From section ?? we can learn that flash is mapped at address 0x08000000. Our MCU has 2MB flash.

<img src="images/mem.svg" />

From the datasheet we can also learn that there are many more memory regions.
Their address ranges are given in the section ?? "Memory Map". For example,
there is a "GPIOA" region that starts at 0x48000000.

These memory regions correspond to a different "peripherals" inside the MCU -
a piece of silicon circuitry that make certain pins behave in a special way.
A peripheral memory region is a collection of 32-bit **registers**. Each
register is a 4-byte memory range at a certain address, that maps to a certain
function of the given peripheral. By writing values into a register - in other
words, by writing a 32-bit value at a certain memory address, we can control
how given peripheral should behave. By reading registers, we can read back
peripheral's data or configuration.

There are many different peripherals. One of the simpler ones are GPIO
(General Purpose Input Output), which allow user to set MCU pins
into "output mode" and set high or low voltage on them. Or, set pins into
an "input mode" and read voltage values from them. There is a UART peripheral
which can transmit and receive serial data over two pins using serial protocol.
There are many other peripherals.

Often, there are multiple "instances" of the same peripheral, for example
GPIOA, GPIOB, ... which control different set of MCU pins. Likewise, there
could be UART1, UART2, ... which allow to implement multiple UART channels.
On Nucleo-L4R5, there are several GPIO and UART peripherals.

For example, GPIOA peripheral starts at 0x48000000, and we can find GPIO register description in section 8.4.12. The datasheet says that `GPIOA_MODER` register has offset 0, that
means that it's address is `0x48000000 + 0`, and this is the format of the
register:

<img src="images/moder.png" style="max-width: 100%" />

The datasheet shows that the 32-bit MODER register is a collection of 2-bit
values, 16 in total. Therefore, one MODER register controls 16 physical pins,
Bits 0-1 control pin 0, bits 2-3 control pin 1, and so on. The 2-bit value
encodes pin mode: 0 means input, 1 means output, 2 means "alternate function" -
some specific behavior described elsewhere, and 3 means analog. Since the
peripheral name is "GPIOA", then pins are named "A0", "A1", etc. For peripheral
"GPIOB", pin naming would be "B0", "B1", ...

If we write 32-bit value `0` to the register MODER, we'll set all 16 pins,
from A0 to A15, to input mode:

```c
  * (volatile uint32_t *) (0x48000000 + 0) = 0;  // Set A0-A15 to input mode
```

Note the `volatile` specifier. Its meaning will be covered later.  By setting
individual bits, we can selectively set specific pins to a desired mode. For
example, this snippet sets pin A3 to output mode:

```c
  * (volatile uint32_t *) (0x48000000 + 0) &= ~(3 << 6);  // CLear bit range 6-7
  * (volatile uint32_t *) (0x48000000 + 0) |= 1 << 6;     // Set bit range 6-7 to 1
```

Let me explain those bit operations. Our goal is to set bits 6-7, which are
responsible for the pin 3 of GPIOA peripheral, to a specific value (1, in our
case). This is done in two steps. First, we must clear the current value of
bits 6-7, because it may hold some value already. Then we must set bits 6-7
to the value we want.

So, first, we must set bit range 6-7 to zero. How do we set
a number of bits to zero? In four steps:

- Get a number that has N contiguous bits set:
   - 1 for 1 bit:  `0b1`,
   - 3 for 2 bits: `0b11`,
   - 7 for 3 bits: `0b111`,
   - 15 for 4 bits: `0b1111`,
   - and so on: generally, for N bits, the number is `2^N - 1`
  So, for 2 bits it is number `3`, or `0b00000000000000000000000000000011`
- Shift that number left. If we need to set bits X-Y, then shift on X positions
  left. In our case, shift on a 6 positions left: `(3 << 6)`, or
  `0b00000000000000000000000011000000`
- Invert the number: turn zeros to ones, and ones to zeroes:
  `~(3 << 6)`, or `0xb11111111111111111111111100111111`
- Now, perform a "logical AND" operation of the register with our number. Bits
  6-7, AND-ed with 0, will give zero - that's what we want! All other bits,
  AND-ed with 1, will retain their current value: `REG &= ~(3 << 6)`. Retaining
  values of all other bits is important: we don't want to change other settings
  in other bit ranges!

So, in general, if we want to clear bits X-Y (set them to zero), do:

```c
PERIPHERAL->REGISTER &= ~(NUMBER_WITH_N_BITS << X);
```

And, finally, we want to set a given bit range to the value we want. We
shift that value X positions left, and OR with the current value of the whole
register:

```c
PERIPHERAL->REGISTER |= VALUE << X;
```

Now, it should be clear to you, dear reader, the meaning of these two lines,
which set bits 6-7 of the GPIOA MODER register to the value of 1 (output).

```c
  * (volatile uint32_t *) (0x48000000 + 0) &= ~(3 << 6);  // CLear bit range 6-7
  * (volatile uint32_t *) (0x48000000 + 0) |= 1 << 6;     // Set bit range 6-7 to 1
```

Some registers are not mapped to the MCU peripherals, but they are mapped to
the ARM CPU configuration and control. For example, there is a "Reset at clock
control" unit (RCC), described in section ?? of the datasheet. It describes
registers that allow to set systems clock and other things.

## Human-readable peripherals programming

In the previous section we have learned that we can read and write peripheral
register by direct accessing certain memory addresses. Let's look at the
snippet that sets pin A3 to output mode:

```c
  * (volatile uint32_t *) (0x48000000 + 0) &= ~(3 << 6);  // CLear bit range 6-7
  * (volatile uint32_t *) (0x48000000 + 0) |= 1 << 6;     // Set bit range 6-7 to 1
```

That is pretty cryptic. Without extensive comments, such code would be quite
hard to understand. We can rewrite this code to a much more readable form.  The
idea is to represent the whole peripheral as a structure that contains 32-bit
fields. Let's see what registers exist for the GPIO peripheral in the section
?? of the datasheet. They are MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR,
LCKR, AFR, BRR. Their offsets are with offsets 0, 4, 8, etc... . That means we can
represent them as a structure with 32-bit fields, and make a define for GPIOA:

```c
struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
};

#define GPIOA ((struct gpio *) 0x48000000)
```

Then, for setting GPIO pin mode, we can define a function:

```c
// Enum values are per datasheet: 0, 1, 2, 3
enum {GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG};

static inline void gpio_set_mode(struct gpio *gpio, uint8_t pin, uint8_t mode) {
  gpio->MODER &= ~(3U << (pin * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (pin * 2);   // Set new mode
}
```
Now, we can rewrite the snippet for A3 like this:

```c
gpio_set_mode(GPIOA, 3 /* pin */, GPIO_MODE_OUTPUT);  // Set A3 to output
```

Our MCU has several GPIO peripherals (also called "banks"): A, B, C, ... K.
From section 2.2.2 (Table 1) we can see that they are 1KB away from each other:
GPIOA is at address 0x48000000, GPIOB is at 0x48000400, and so on:

```c
#define GPIO(bank) ((struct gpio *) (0x40020000 + 0x400 * (bank)))
```

We can create pin numbering that includes the bank and the pin number.
To do that, we use 2-byte `uint16_t` value, where upper byte indicates
GPIO bank, and lower byte indicates pin number:

```c
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)
```

This way, we can specify pins for any GPIO bank:

```c
  uint16_t pin1 = PIN('A', 3);    // A3   - GPIOA pin 3
  uint16_t pin2 = PIN('G', 11);   // G11  - GPIOG pin 11
```

Let's rewrite the `gpio_set_mode()` function to take our pin specification:

```c
static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin)); // GPIO bank
  uint8_t n = PINNO(pin);                 // Pin number
  gpio->MODER &= ~(3U << (n * 2));        // Clear existing setting
  gpio->MODER |= (mode & 3) << (n * 2);   // Set new mode
}
```

Now the code for A3 is self-explanatory:

```c
  uint16_t pin = PIN('A', 3);            // Pin A3
  gpio_set_mode(pin, GPIO_MODE_OUTPUT);  // Set to output
```

Note that we have created a useful initial API for the GPIO peripheral. Other
peripherals, like UART (serial communication) and others - can be implemented
in a similar way. This is a good programming practice that makes code
self-explanatory and human readable.

## MCU boot and vector table

When STM32L4R5 MCU boots, it reads a so-called "vector table" from the
beginning of flash memory. A vector table is a concept common to all ARM MCUs.
That is a array of 32-bit addresses of interrupt handlers. First 16 entries
are reserved by ARM and are common to all ARM MCUs. The rest of interrupt
handlers are specific to the given MCU - these are interrupt handlers for
peripherals. Simpler MCUs with few peripherals have few interrupt handlers,
and more complex MCUs have many.

Vector table for STM32L4R5 is documented in Table 76. From there we can learn
that there are 95 peripheral handlers, in addition to the standard 16.

At this point, we are interested in the first two entries of the vector table,
because they play a key role in the MCU boot process. Those two first values
are: initial stack pointer, and an address of the boot function to execute
(a firmware entry point).

So now we know, that we must make sure that our firmware should be composed in
a way that the 2nd 32-bit value in the flash should contain an address of
out boot function. When MCU boots, it'll read that address from flash, and
jump to our boot function.

## Minimal firmware

Let's create a file `main.c`, and specify our boot function that initially does
nothing (falls into infinite loop), and specify a vector table that contains 16
standard entries and 95 STM32 entries. In your editor of choice, create
`main.c` file and copy/paste the following into `main.c` file:

```c
// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  for (;;) (void) 0;  // Infinite loop
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 91 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 95])(void) = {
  _estack, _reset
};
```

For function `_reset()`, we have used GCC-specific attributes `naked` and
`noreturn` - they mean, standard function's prologue and epilogue should not
be created by the compiler, and that function does not return.

The `void (*tab[16 + 95])(void)` expression means: define an array of 16 + 95
pointers to functions, that return nothing (void) and take to arguments. Each
such function is an IRQ handler (Interrupt ReQuest handler). An array of those
handlers is called a vector table.

The vector table `tab` we put in a separate section called `.vectors` - that we
need later to tell the linker to put that section right at the beginning of the
generated firmware - and consecutively, at the beginning of flash memory. The
first two entries are: the value of the stack pointer register, and the
firmware's entry point.  We leave the rest of vector table filled with zeroes.

### Compilation

Let's compile our code. Start a terminal (or a command prompt on Windows) and execute:

```sh
$ arm-none-eabi-gcc -mcpu=cortex-m4 main.c -c
```

That works! The compilation produced a file `main.o` which contains
our minimal firmware that does nothing.  The `main.o` file is in ELF binary
format, which contains several sections. Let's see them:

```sh
$ arm-none-eabi-objdump -h main.o
...
Idx Name          Size      VMA       LMA       File off  Algn
  0 .text         00000002  00000000  00000000  00000034  2**1
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
  1 .data         00000000  00000000  00000000  00000036  2**0
                  CONTENTS, ALLOC, LOAD, DATA
  2 .bss          00000000  00000000  00000000  00000036  2**0
                  ALLOC
  3 .vectors      000001ac  00000000  00000000  00000038  2**2
                  CONTENTS, ALLOC, LOAD, RELOC, DATA
...
```

Note that VMA/LMA addresses for sections are set to 0 - meaning, `main.o`
is not yet a complete firmware, because it does not contain the information
where those section should be loaded in the address space. We need to use
a linker to produce a full firmware `firmware.elf` from `main.o`.

The section .text contains firmware code, in our case it is just a _reset()
function, 2-bytes long - a jump instruction to its own address. There is
an empty `.data` section and an empty `.bss` section
(data that is initialized to zero) . Our firmware will be copied
to the flash region at offset 0x8000000, but our data section should reside
in RAM - therefore our `_reset()` function should copy the contents of the
`.data` section to RAM. Also it has to write zeroes to the whole `.bss`
section. Our `.data` and `.bss` sections are empty, but let's modify our
`_reset()` function anyway to handle them properly.

In order to do all that, we must know where stack starts, and where data and
bss section start. This we can specify in the "linker script", which is a file
with the instructions to the linker, where to put various sections in the
address space, and which symbols to create.

### Linker script

Create a minimal linker script `link.ld`, and copy-paste contents from
[step-0-minimal/link.ld](step-0-minimal/link.ld). Below is the explanation:

```
ENTRY(_reset);
```
This line tells the linker the value of the "entry point" attribute in the
generated ELF header - so this is a duplicate to what a vector table has.  This
is an aid for a debugger (like Ozone, described below) that helps to set a
breakpoint at the beginning of the firmware.  A debugger does not know about a
vector table, so it relies on the ELF header.

```
MEMORY {
  flash(rx)  : ORIGIN = 0x08000000, LENGTH = 2048k
  sram(rwx) : ORIGIN = 0x20000000, LENGTH = 192k  /* remaining 64k in a separate address space */
}
```
This tells the linker that we have two memory regions in the address space,
their addresses and sizes.

```
_estack     = ORIGIN(sram) + LENGTH(sram);    /* stack points to end of SRAM */
```

This tell a linker to create a symbol `estack` with value at the very end
of the RAM memory region. That will be our initial stack value!

```
  .vectors  : { KEEP(*(.vectors)) }   > flash
  .text     : { *(.text*) }           > flash
  .rodata   : { *(.rodata*) }         > flash
```

These lines tell the linker to put vectors table on flash first,
followed by `.text` section (firmware code), followed by the read only
data `.rodata`.

The next goes `.data` section:

```
  .data : {
    _sdata = .;   /* .data section start */
    *(.first_data)
    *(.data SORT(.data.*))
    _edata = .;  /* .data section end */
  } > sram AT > flash
  _sidata = LOADADDR(.data);
```

Note that we tell linker to create `_sdata` and `_edata` symbols. We'll
use them to copy data section to RAM in the `_reset()` function.

Same for `.bss` section:

```
  .bss : {
    _sbss = .;              /* .bss section start */
    *(.bss SORT(.bss.*) COMMON)
    _ebss = .;              /* .bss section end */
  } > sram
```

### Startup code

Now we can update our `_reset()` function. We copy `.data` section to RAM, and
initialise bss section to zeroes. Then, we call main() function - and fall into
infinite loop in case if main() returns:

```c
int main(void) {
  return 0; // Do nothing so far
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // memset .bss to zero, and copy .data section to RAM region
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  main();             // Call main()
  for (;;) (void) 0;  // Infinite loop in the case if main() returns
}
```

The following diagram visualises how `_reset()` initialises .data and .bss:

![](images/mem2.svg)

The `firmware.bin` file is just a concatenation of the three sections:
`.vectors` (IRQ vector table), `.text` (code) and `.data` (data).  Those
sections were built according to the linker script: `.vectors` lies at the very
beginning of flash, then `.text` follows immediately after, and `.data` lies
far above. Addresses in `.text` are in the flash region, and addresses in
`.data` are in the RAM region.  If some function has address e.g. `0x8000100`,
then it it located exactly at that address on flash. But if the code accesses
some variable in the `.data` section by the address e.g. `0x20000200`, then
there is nothing at that address, because at boot, `.data` section in the
`firmware.bin` resides in flash! That's why the startup code must relocate
`.data` section from flash region to the RAM region.

Now we are ready to produce a full firmware file `firmware.elf`:

```sh
$ arm-none-eabi-gcc -T link.ld -nostdlib main.o -o firmware.elf
```

Let's examine sections in firmware.elf:

```sh
$ arm-none-eabi-objdump -h firmware.elf
...
Idx Name          Size      VMA       LMA       File off  Algn
  0 .vectors      000001ac  08000000  08000000  00010000  2**2
                  CONTENTS, ALLOC, LOAD, DATA
  1 .text         00000058  080001ac  080001ac  000101ac  2**2
                  CONTENTS, ALLOC, LOAD, READONLY, CODE
...
```

Now we can see that the .vectors section will reside at the very beginning of
flash memory at address 0x8000000, then the .text section right after it, at
0x80001ac. Our code does not create any variables, so there is no data section.

## Flash firmware

We're ready to flash this firmware! First, extract sections from the
firmware.elf into a single contiguous binary blob:

```sh
$ arm-none-eabi-objcopy -O binary firmware.elf firmware.bin
```

And use `st-link` utility to flash the firmware.bin. Plug your board to the
USB, and execute:

```sh
$ st-flash --reset write firmware.bin 0x8000000
```

Done! We've flashed a firmware that does nothing.

## Makefile: build automation

Instead of typing those compilation, linking and flashing commands, we can
use `make` command line tool to automate the whole process. `make` utility
uses a configuration file named `Makefile` where it reads instructions
how to execute actions. This automation is great because it also documents the
process of building firmware, used compilation flags, etc.

There is a great Makefile tutorial at https://makefiletutorial.com - for those
new to `make`, I suggest to take a look. Below, I list the most essential
concepts required to understand our simple bare metal Makefile. Those who
already familiar with `make`, can skip this section.

The `Makefile` format is simple:

```make
action1:
	command ...     # Comments can go after hash symbol
	command ....    # IMPORTANT: command must be preceded with the TAB character

action2:
	command ...     # Don't forget about TAB. Spaces won't work!
```

Now, we can invoke `make` with the action name (also called *target*) to execute
a corresponding action:

```sh
$ make action1
```

It is possible to define variables and use them in commands. Also, actions
can be file names that needs to be created:

```make
firmware.elf:
	COMPILATION COMMAND .....
```

And, any action can have a list of dependencies. For example, `firmware.elf`
depends on our source file `main.c`. Whenever `main.c` file changes, the
`make build` command rebuilds `firmware.elf`:

```
build: firmware.elf

firmware.elf: main.c
	COMPILATION COMMAND
```

Now we are ready to write a Makefile for our firmware. We define a `build`
action / target:

```make
CFLAGS  ?=  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion \
            -Wformat-truncation -fno-common -Wconversion \
            -g3 -Os -ffunction-sections -fdata-sections -I. \
            -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 $(EXTRA_CFLAGS)
LDFLAGS ?= -T link.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=$@.map
SOURCES = main.c  

build: firmware.elf

firmware.elf: $(SOURCES)
	arm-none-eabi-gcc $(SOURCES) $(CFLAGS) $(LDFLAGS) -o $@
```

There, we define compilation flags. The `?=` means that's a default value;
we could override them from the command line like this:

```sh
$ make build CFLAGS="-O2 ...."
```

We specify `CFLAGS`, `LDFLAGS` and `SOURCES` variables.
Then we tell `make`: if you're told to `build`, then create a `firmware.elf`
file. It depends on the `main.c` file, and to create it, start
`arm-none-eabi-gcc` compiler with a given flags. `$@` special variable
expands to a target name - in our case, `firmware.elf`.

Let's call `make`:

```
$ make build
arm-none-eabi-gcc main.c  -W -Wall -Wextra -Werror -Wundef -Wshadow -Wdouble-promotion -Wformat-truncation -fno-common -Wconversion -g3 -Os -ffunction-sections -fdata-sections -I. -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16  -Tlink.ld -nostartfiles -nostdlib --specs nano.specs -lc -lgcc -Wl,--gc-sections -Wl,-Map=firmware.elf.map -o firmware.elf
```

If we run it again:

```sh
$ make build
make: Nothing to be done for `build'.
```

The `make` utility examines modification times for `main.c` dependency and
`firmware.elf` - and does not do anything if `firmware.elf` is up to date.
But if we change `main.c`, then next `make build` will recompile:

```sh
$ touch main.c # Simulate changes in main.c
$ make build
```

Now, what is left - is the `flash` target:


```make
firmware.bin: firmware.elf
	$(DOCKER) $(CROSS)-objcopy -O binary $< $@

flash: firmware.bin
	st-flash --reset write $(TARGET).bin 0x8000000
```

That's it! Now, `make flash` terminal command creates a `firmware.bin` file,
and flashes it to the board. It'll recompile the firmware if `main.c` changes,
because `firmware.bin` depends on `firmware.elf`, and it in turn depends on
`main.c`. So, now the development cycle would be these two actions in a loop:

```sh
# Develop code in main.c
$ make flash
```

It is a good idea to add a clean target to remove build artifacts:


```
clean:
	rm -rf firmware.*
```

A complete project source code you can find in [step-0-minimal](step-0-minimal) folder.