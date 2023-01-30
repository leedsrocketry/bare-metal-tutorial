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

## Memory and registers
The 32-bit address space of the MCU is divided by regions. For example, some region of memory is mapped to the internal MCU flash at a specific address. Firmware code instructions are read and executed by reading from that memory region. Another region is RAM, which is also mapped to a specific address. We can read and write any values to the RAM region.

From STM32L4R5 datasheet, we can take a look at section ?? and learn that RAM region starts at address 0x20000000 and has size of 192KB. From section ?? we can learn that flash is mapped at address 0x08000000. Our MCU has 2MB flash.
