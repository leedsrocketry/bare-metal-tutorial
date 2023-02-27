#include "mcu.h"

static volatile uint32_t s_ticks;
void SysTick_Handler(void) {
  s_ticks++;
}

int main(void) {
  uint16_t led = PIN('B', 7);                         // Blue LED
  systick_init(FREQ / 1000);                          // Tick every 1 ms
  gpio_set_mode(led, GPIO_MODE_OUTPUT);               // Set blue LED to output mode
  
  pwr_vdd2_init();
  uart_init(LUART1, 115200);                          // Initialise UART; 
  uint32_t timer = 0, period = 500;                   // Declare timer and 500ms period 

  for (;;) {
    if (timer_expired(&timer, period, s_ticks)) {
      static bool on;                                 // This block is executed
      gpio_write(led, on);                            // Every `period` milliseconds
      on = !on;                                       // Toggle LED state
      printf("Wello LURA!\r\n");                      // Write message
    }
  }
  return 0;
}