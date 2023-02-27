#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x)) // create a 32-bit number 
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

struct rcc {
  volatile uint32_t CR, ICSCR, CFGR, PLLCFGR, PLLSAI1CFGR, PLLSAI2CFGR, CIER, CIFR,
      CICR, RESERVE0, AHB1RSTR, AHB2RSTR, AHB3RSTR, RESERVE1, APB1RSTR1, APB1RSTR2, APB2RSTR, RESERVE2, AHB1ENR, 
      AHB2ENR, AHB3ENR, RESERVE3, APB1ENR1, APB1ENR2, APB2ENR, RESERVE4, AHB1SMENR, AHB2SMENR, AHB3SMENR, RESERVE5,
      APB1SMENR1, APB1SMENR2, APB2SMENR, RESERVE6, CCIPR, BDCR, CSR, CRRCR, CCIPR2, RESERVE7, DLYCFGR;
};
#define RCC ((struct rcc *) 0x40021000)

struct gpio {
  volatile uint32_t MODER, OTYPER, OSPEEDR, PUPDR, IDR, ODR, BSRR, LCKR, AFR[2], BRR;
};
#define GPIO(bank) ((struct gpio *) (0x48000000 + 0x400 * (bank)))

// Enum values are per datasheet: 0, 1, 2, 3
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

int main(void) {
  uint16_t led = PIN('B', 7);            // Blue LED
  RCC->AHB2ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  
  for (;;) {
    gpio_write(led, true);
    spin(999999);
    gpio_write(led, false);
    spin(999999);
  }
  return 0;
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

extern void _estack(void);  // Defined in link.ld

// 16 standard and 95 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 95])(void) = {_estack, _reset};
