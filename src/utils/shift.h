#pragma once

#include <Arduino.h>

#include "gio/gio.h"

namespace gio::shift {

// read
// прочитать пакет
void read(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// прочитать байт
uint8_t read_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t delay = 0);

// прочитать пакет + cs пин
void read_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// прочитать байт + cs пин
uint8_t read_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t delay = 0);

// send
// отправить пакет
void send(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// отправить байт
void send_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t data, uint8_t delay = 0);

// отправить пакет + cs пин
void send_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// отправить байт + cs пин
void send_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t data, uint8_t delay = 0);

}  // namespace gio::shift

/*
us byte (MHz)
|         | shiftOut     | gio_send | gio_send (non-const) |
|---------|--------------|----------|----------------------|
| AVR     | 100 (0.075)  | 6 (1.3)  | 11 (0.7)             |
*/