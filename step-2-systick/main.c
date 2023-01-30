// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include <inttypes.h>
#include <stdbool.h>

#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

// SysTick Control and Status Register; Reload Value; Current; Calibration
struct systick {
  volatile uint32_t STCSR, STRVR, STCVR, STCR;
};
#define SYSTICK ((struct systick *) 0xe000e010)  // 4.1 System control registers

struct rcc {
  volatile uint32_t CR, ICSCR, CFGR, PLLCFGR, PLLSAI1CFGR, PLLSAI2CFGR, CIER, CIFR,
      CICR, RESERVE0, AHB1RSTR, AHB2RSTR, AHB3RSTR, RESERVE1, APB1RSTR1, APB1RSTR2, APB2RSTR, RESERVE2, AHB1ENR, 
      AHB2ENR, AHB3ENR, RESERVE3, APB1ENR1, APB1ENR2, APB2ENR, RESERVE4, AHB1SMENR, AHB2SMENR, AHB3SMENR, RESERVE5,
      APB1SMENR1, APB1SMENR2, APB2SMENR, RESERVE6, CCIPR, BDCR, CSR, CRRCR, CCIPR2, RESERVE7, DLYCFGR;
};
#define RCC ((struct rcc *) 0x40021000)

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->STRVR = ticks - 1;
  SYSTICK->STCVR = 0;
  SYSTICK->STCSR = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(0);                     // Enable SYSCFG
}

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

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}

int main(void) {
  uint16_t led = PIN('B', 7);            // Blue LED
  RCC->AHB2ENR |= BIT(PINBANK(led));     // Enable GPIO clock for LED
  systick_init(4000000 / 000);          // Tick every 1 ms; it starts at 4MHz
  gpio_set_mode(led, GPIO_MODE_OUTPUT);  // Set blue LED to output mode
  uint32_t timer, period = 250;          // Declare timer and 250ms period

  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on = true;      // This block is executed
      gpio_write(led, on);  // Every `period` milliseconds
      on = !on;             // Toggle LED state
    }
    // Here we could perform other activities!
  }
  return 0;
}

// Startup code
__attribute__((naked, noreturn)) void _reset(void) {
  // Initialise memory
  extern long _sbss, _ebss, _sdata, _edata, _sidata;
  for (long *src = &_sbss; src < &_ebss; src++) *src = 0;
  for (long *src = &_sdata, *dst = &_sidata; src < &_edata;) *src++ = *dst++;

  // Call main()
  main();
  for (;;) (void) 0;  // Infinite loop
}

extern void _estack(void);  // Defined in link.ld

// 16 standard and 95 STM32-specific handlers
__attribute__((section(".vectors"))) void (*tab[16 + 95])(void) = {
    _estack, _reset, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, SysTick_Handler};
