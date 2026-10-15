#include "main.h"

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  uart_init();

  while (1) {
  }

  return 0;
}