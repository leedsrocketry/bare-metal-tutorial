// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#include <inttypes.h>
#include <stdbool.h>
#include <stdlib.h>

#define FREQ 4000000  // CPU frequency, 4 Mhz
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

struct systick {
  volatile uint32_t CTRL, LOAD, VAL, CALIB;
};
#define SYSTICK ((struct systick *) 0xe000e010)  // 2.2.2

struct rcc {
  volatile uint32_t CR, ICSCR, CFGR, PLLCFGR, PLLSAI1CFGR, PLLSAI2CFGR, CIER, CIFR,
      CICR, RESERVE0, AHB1RSTR, AHB2RSTR, AHB3RSTR, RESERVE1, APB1RSTR1, APB1RSTR2, APB2RSTR, RESERVE2, AHB1ENR, 
      AHB2ENR, AHB3ENR, RESERVE3, APB1ENR1, APB1ENR2, APB2ENR, RESERVE4, AHB1SMENR, AHB2SMENR, AHB3SMENR, RESERVE5,
      APB1SMENR1, APB1SMENR2, APB2SMENR, RESERVE6, CCIPR, BDCR, CSR, CRRCR, CCIPR2, RESERVE7, DLYCFGR;
};
#define RCC ((struct rcc *) 0x40021000)

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SYSTICK->LOAD = ticks - 1;
  SYSTICK->VAL = 0;
  SYSTICK->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(0);                   // Enable SYSCFG
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
  RCC->AHB2ENR |= BIT(PINBANK(pin));       // Enable GPIO clock
  gpio->MODER &= ~(3U << (n * 2));         // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);   // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  struct gpio *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                      // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

static inline void gpio_write(uint16_t pin, bool val) {
  struct gpio *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}

// Datasheet STM32L4 50.8.15 USART register map
struct uart {
  volatile uint32_t CR1, CR2, CR3, BRR, GTPR, RTOR, RQR, ISR, ICR, RDR, TDR, PRESC;
};

// Datasheet STM32L4 2.2 Memory Organization
#define UART1 ((struct uart *) 0x40013800)
#define UART2 ((struct uart *) 0x40004400)
#define UART3 ((struct uart *) 0x40004800)

static inline void uart_init(struct uart *uart, unsigned long baud) {
  uint8_t af = 7;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins

  if (uart == UART1) RCC->APB2ENR  |= BIT(14);  // Datasheet STM32L4 page 296
  if (uart == UART2) RCC->APB1ENR1 |= BIT(17);  // Datasheet STM32L4 page 291
  if (uart == UART3) RCC->APB1ENR1 |= BIT(18);  // Datasheet STM32L4 page 291

  if (uart == UART3) af = 7, tx = PIN('D', 8), rx = PIN('D', 9); // Nucleo User Manual page 40

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;                                // Disable this UART                              
  uart->BRR = FREQ / baud;                      // FREQ is a CPU frequency
  uart->CR1 |= BIT(0) | BIT(2) | BIT(3);        // Set UE, RE, TE Datasheet 50.8.1 
}

static inline void uart_write_byte(struct uart *uart, uint8_t byte) {
  uart->RDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1);    // Datasheet STM32L4 50.8.10 USART status register (USART_ISR) 
}

static inline void uart_write_buf(struct uart *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(struct uart *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready Datasheet 50.8.10
}

static inline uint8_t uart_read_byte(struct uart *uart) {
  return (uint8_t) (uart->RDR & 255);
}

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

// t: expiration time, prd: period, now: current time. Return true if expired
bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                     // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                    // Firt poll? Set expiration
  if (*t > now) return false;                     // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;   // Next expiration time
  return true;                                    // Expired, return true
}

int main(void) {
  uint16_t led = PIN('B', 7);                     // Blue LED
  systick_init(4000000 / 4000);                   // Tick every 1 ms
  gpio_set_mode(led, GPIO_MODE_OUTPUT);           // Set blue LED to output mode
  uart_init(UART3, 9600);                       // Initialise UART
  uint32_t timer, period = 500;                   // Declare timer and 500ms period
  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;                             // This block is executed
      gpio_write(led, on);                        // Every `period` milliseconds
      on = !on;                                   // Toggle LED state
      uart_write_buf(UART3, "hi\r\n", 4);         // Write message
    }
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