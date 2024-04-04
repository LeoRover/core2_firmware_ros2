#pragma once

#include <DigitalOut.h>

#include <cstdint>

struct GPIO {
  GPIO_TypeDef* port;
  uint16_t pin;
};

inline void gpio_set(mbed::DigitalOut& gpio) {
  gpio = 0;
}

inline void gpio_reset(mbed::DigitalOut& gpio) {
  gpio = 1;
}

inline void gpio_toggle(mbed::DigitalOut& gpio) {
  gpio = gpio == 0 ? 1 : 0;
}

inline void reset() {
  NVIC_SystemReset();
}

inline void delay(uint32_t delay_ms) {
  ThisThread::sleep_for(delay_ms);
}
