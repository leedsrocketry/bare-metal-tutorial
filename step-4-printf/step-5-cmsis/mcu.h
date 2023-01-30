// Copyright (c) 2022 Cesanta Software Limited
// All rights reserved

#pragma once

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

// https://github.com/STMicroelectronics/cmsis_device_l4/blob/master/Include/system_stm32l4xx.h
#include "stm32l4r5xx.h"

#define FREQ 4000000  // CPU frequency, 4 Mhz
#define BIT(x) (1UL << (x))
#define PIN(bank, num) ((((bank) - 'A') << 8) | (num))
#define PINNO(pin) (pin & 255)
#define PINBANK(pin) (pin >> 8)

static inline void spin(volatile uint32_t count) {
  while (count--) asm("nop");
}

static inline void systick_init(uint32_t ticks) {
  if ((ticks - 1) > 0xffffff) return;  // Systick timer is 24 bit
  SysTick->LOAD = ticks - 1;
  SysTick->VAL = 0;
  SysTick->CTRL = BIT(0) | BIT(1) | BIT(2);  // Enable systick
  RCC->APB2ENR |= BIT(14);                   // Enable SYSCFG
}

#define GPIO(bank) ((GPIO_TypeDef *) (GPIOA_BASE + 0x400U * (bank)))
enum { GPIO_MODE_INPUT, GPIO_MODE_OUTPUT, GPIO_MODE_AF, GPIO_MODE_ANALOG };

static inline void gpio_set_mode(uint16_t pin, uint8_t mode) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                       // Pin number
  RCC->AHB1ENR |= BIT(PINBANK(pin));        // Enable GPIO clock
  gpio->MODER &= ~(3U << (n * 2));          // Clear existing setting
  gpio->MODER |= (mode & 3U) << (n * 2);    // Set new mode
}

static inline void gpio_set_af(uint16_t pin, uint8_t af_num) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));  // GPIO bank
  int n = PINNO(pin);                       // Pin number
  gpio->AFR[n >> 3] &= ~(15UL << ((n & 7) * 4));
  gpio->AFR[n >> 3] |= ((uint32_t) af_num) << ((n & 7) * 4);
}

static inline void gpio_write(uint16_t pin, bool val) {
  GPIO_TypeDef *gpio = GPIO(PINBANK(pin));
  gpio->BSRR |= (1U << PINNO(pin)) << (val ? 0 : 16);
}

#define UART1 USART1
#define UART2 USART2
#define UART3 USART3

static inline void uart_init(USART_TypeDef *uart, unsigned long baud) {
  uint8_t af = 0;           // Alternate function
  uint16_t rx = 0, tx = 0;  // pins
  
  if (uart == UART1) RCC->APB2ENR  |= BIT(14);  // Datasheet STM32L4 page 296
  if (uart == UART2) RCC->APB1ENR1 |= BIT(17);  // Datasheet STM32L4 page 291
  if (uart == UART3) RCC->APB1ENR1 |= BIT(18);  // Datasheet STM32L4 page 291
  if (uart == UART3) af = 7, tx = PIN('D', 8), rx = PIN('D', 9); // Nucleo User Manual page 40

  gpio_set_mode(tx, GPIO_MODE_AF);
  gpio_set_af(tx, af);
  gpio_set_mode(rx, GPIO_MODE_AF);
  gpio_set_af(rx, af);
  uart->CR1 = 0;                            // Disable this UART
  uart->BRR = FREQ / baud;                  // FREQ is a CPU frequency
  uart->CR1 |= BIT(0) | BIT(2) | BIT(3);    // Set UE, RE, TE
}

static inline void uart_write_byte(USART_TypeDef *uart, uint8_t byte) {
  uart->RDR = byte;
  while ((uart->ISR & BIT(7)) == 0) spin(1); // Datasheet STM32L4 50.8.10 USART status register (USART_ISR)
}

static inline void uart_write_buf(USART_TypeDef *uart, char *buf, size_t len) {
  while (len-- > 0) uart_write_byte(uart, *(uint8_t *) buf++);
}

static inline int uart_read_ready(USART_TypeDef *uart) {
  return uart->ISR & BIT(5);  // If RXNE bit is set, data is ready Datasheet 50.8.10
}

static inline uint8_t uart_read_byte(USART_TypeDef *uart) {
  return (uint8_t) (uart->RDR & 255);
}

static inline bool timer_expired(uint32_t *t, uint32_t prd, uint32_t now) {
  if (now + prd < *t) *t = 0;                    // Time wrapped? Reset timer
  if (*t == 0) *t = now + prd;                   // Firt poll? Set expiration
  if (*t > now) return false;                    // Not expired yet, return
  *t = (now - *t) > prd ? now + prd : *t + prd;  // Next expiration time
  return true;                                   // Expired, return true
}
