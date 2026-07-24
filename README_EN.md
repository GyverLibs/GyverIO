This is an automatic translation, may be incorrect in some places. See sources and examples!

# Gyverio
Fast functions for working with AVR Pin (a full list look at Gio_avr.h), ESP8266, ESP32
- acceleration on average by 20-30 times, the final time for all architectures is almost the same
- Classes for quick PIN control
- Separate processing of cases of constant and uncontrolled pins for AVR
- Fast implementation of Shiftin/Shiftout
- Lightweight software `SoftWire` and `SoftSPI`
- Universal class Hard Spi / Soft SPI for use in libraries

## Speed
### measurement conditions
||Version |Frequency |
| --------- | -------- | -------- |
|AVR |1.8.19 |16 |
|ESP8266 |3.1.2 |80 |
|ESP32 |2.0.11 |240 |
|ESP32C3 |2.0.11 |80 |

### GPIO (US)
||Write NC |Write |Read NC |Read |Mode NC |Mode |
| ------------- | ---------- | ----------------- | ------------------------------| ------------ | --------- | ------------
|AVR Ardu |** 5.3 ** |5.3 |** 4.8 ** |4.8 |** 3.3 ** |3.3 |
|AVR Gio |1.6 |*** 0.125 *** |1.75 |*** 0.075 *** |1.8 |*** 0.125 *** |
||||||||
|ESP8266 Ardu |** 1.5 ** |1.5 |** 0.54 ** |0.54 |** 1.4 ** |1.4 |
|ESP8266 Gio |0.29 |*** 0.08 *** |0.5 |*** 0.17 *** |1.29 |*** 0.58 *** |
||||||||
|ESP32 Ardu |** 0.33 ** |0.33 |** 0.124 ** |0.124 |** 16 ** |16 |
|ESP32 Gio |0.04 |*** 0.04 *** |0.085 |*** 0.085 *** |0.126 |*** 0.08 *** |
||||||||
|ESP32C3 Ardu |** 0.91 ** |0.91 |** 0.25 ** |0.25 |** 21 ** |21 |
|ESP32C3 Gio |0.05 |*** 0.05 *** |0.4 |*** 0.08 *** |0.49 |*** 0.08 *** |

> * Nc * - Pins are not constants

> ** Fat ** highlighted the worst time (Arduino not constants), *** with a fat italics *** - the best (gio constants)

### Shift (MHZ)
||shiftOut |gio::shift |
| -------- | ---------- | -----------------------------------
|AVR NC |0.06 |0.66 |
|AVR |0.06 |1.3 |
|ESP8266 |0.2 |1.1 |
|ESP32 |0.96 |6 |
|ESP32C3 |0.35 |2.6 |

> * Nc * - Pins are not constants

## compatibility
Compatible with all Arduino platforms. Unsupported platforms fall back to Arduino functions.
- On ESP8266 and ESP32, fast `pinMode()` / `mode()` handles only `INPUT` and `OUTPUT`; other modes fall back to the standard `pinMode()`.

## Content
- [documentation] (#docs)
- [use] (#usage)
- [versions] (#varsions)
- [installation] (# Install)
- [bugs and feedback] (#fedback)

<a id="docs"> </a>
## Documentation
### gio
Fast pin functions.

```cpp
int gio::read(uint8_t P);
void gio::high(uint8_t P);
void gio::low(uint8_t P);
void gio::write(uint8_t P, uint8_t V);
void gio::toggle(uint8_t P);
void gio::mode(uint8_t P, uint8_t V);
void gio::init(uint8_t P, uint8_t V = INPUT);
```

Fast functions expect valid pin numbers. Invalid pin checks are intentionally omitted to keep the code small and fast.

### gio::PinIO
A class for fast access to one pin whose number is selected at runtime.

```cpp
gio::PinIO pin(5, OUTPUT);
pin.high();
pin.low();
pin.write(1);
pin.toggle();
int value = pin.read();
```

On AVR `PinIO` stores `DDR/PORT/PIN` registers and can switch mode with `mode()`.

### gio::PinSingle
A lighter one-mode pin class. On AVR it stores only one register: `PORT` for `OUTPUT`, or `PIN` for `INPUT/INPUT_PULLUP`.

```cpp
gio::PinSingle out(5, OUTPUT);
out.high();
out.low();
out.write(1);

gio::PinSingle inp(6, INPUT_PULLUP);
int value = inp.read();
```

`PinSingle` is not intended for changing mode after `init()`. Use `write/high/low/toggle/read` for `OUTPUT` pins and `read()` for `INPUT` or `INPUT_PULLUP` pins.

### gio::PinT
Template pin class for a pin known at compile time.

```cpp
gio::PinT<5> pin(OUTPUT);
pin.high();
pin.low();
pin.write(1);
pin.toggle();
int value = pin.read();
```

### gio::shift
Fast shiftIn/shiftOut analogue.

```cpp
bool gio::shift::read(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);
uint8_t gio::shift::read_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t delay = 0);
bool gio::shift::read_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);
uint8_t gio::shift::read_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t delay = 0);

void gio::shift::send(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);
void gio::shift::send_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t data, uint8_t delay = 0);
void gio::shift::send_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);
void gio::shift::send_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t data, uint8_t delay = 0);
```

The `order` parameter can be:
- `LSBFIRST`/`LSB_NORMAL` - LSB first, normal byte order
- `MSBFIRST`/`MSB_NORMAL` - MSB first, normal byte order
- `LSB_REVERSE` - LSB first, reverse byte order
- `MSB_REVERSE` - MSB first, reverse byte order

#### Note
- `delay` is in microseconds. For example, `1` us limits speed to about 1 MHz, `2` us to about 500 kHz
- Pins should be configured as `output` yourself before sending (when starting a program for example)

### gio::SSPI
Universal class of software and hardware SPI with optimization of the number of variables for Pin

```cpp
SSPI<0, freq> spi;                  // hardware without CS
SSPI<0, freq, cs> spi;              // hardware with template CS
SSPI<0, freq> spi(cs);              // hardware with runtime CS
SSPI<1, freq, cs, dt, clk> spi;     // software with template pins
SSPI<1, freq> spi(cs, dt, clk);     // software with runtime pins
```

Use a separate include for `SSPI`:

```cpp
#include <GyverIO_SPI.h>
```

### SoftSPI
Minimal software SPI master, mode 0, MSB first. Uses `gio::PinSingle`.

```cpp
SoftSPI spi;
spi.begin(miso, mosi, clk);
spi.setClock(1000000);
spi.write(0x9f);
spi.write(data, len);
int value = spi.read();       // dummy 0xff on MOSI
spi.read(data, len);
uint8_t rx = spi.transfer(tx);
spi.transfer(data, len);      // in-place transfer
```

Chip select is controlled separately.

### SoftWire
Minimal software I2C master with a Wire-like API. Uses `gio::PinIO`, supports clock stretching timeout and bus recovery in `begin()`.

```cpp
SoftWire wire;
wire.begin(sda, scl);
wire.setClock(100000);
wire.beginTransmission(0x40);
wire.write(0x01);
wire.write(data, len);
uint8_t status = wire.endTransmission();
uint8_t count = wire.requestFrom(0x40, len);
int value = wire.read();
wire.read(data, len);
wire.end();
```

`endTransmission()` status codes: `0` success, `2` address NACK, `3` data NACK, `4` bus error or timeout.

### compilation settings
```cpp
#define GIO_USE_ARDUINO // Disable fast functions and use Arduino functions
#define GIO_NO_MASK     // Disable AVR mask IO in PinIO/PinSingle/shift
```

<a id="usage"> </a>
## Usage

```cpp
#include <GyverIO.h>

gio::write(3, 1);

uint8_t data[] = {34, 63, 231, 9};
gio::shift::send(3, 4, MSBFIRST, data, 4);
```

```cpp
#include <GyverIO_SPI.h>

SSPI<0, f, cs> spi;
spi.send(0x12);
```

<a id="versions"> </a>
## versions
- V1.0
- V1.1 - AVR Non -Const is 3 times accelerated, the tables are updated
- V1.2 - Fixed a mistake!
- V1.2.1 - Small optimization
- v1.2.2 - added inversion to Shift

<a id="install"> </a>
## Installation
- The library can be found by the name ** gyverio ** and installed through the library manager in:
    - Arduino ide
    - Arduino ide v2
    - Platformio
- [download the library] (https://github.com/gyverlibs/gyverio/archive/refs/heads/main.zip) .Zip archive for manual installation:
    - unpack and put in * C: \ Program Files (X86) \ Arduino \ Libraries * (Windows X64)
    - unpack and put in * C: \ Program Files \ Arduino \ Libraries * (Windows X32)
    - unpack and put in *documents/arduino/libraries/ *
    - (Arduino id) Automatic installation from. Zip: * sketch/connect the library/add .Zip library ... * and specify downloaded archive
- Read more detailed instructions for installing libraries [here] (https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%BD%D0%BE%BE%BE%BED0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Update
- I recommend always updating the library: errors and bugs are corrected in the new versions, as well as optimization and new features are added
- through the IDE library manager: find the library how to install and click "update"
- Manually: ** remove the folder with the old version **, and then put a new one in its place.“Replacement” cannot be done: sometimes files are deleted in new versions,Cranberries that remain when replacing and can lead to errors!

<a id="feedback"> </a>
## bugs and feedback
Create ** Issue ** when you find the bugs, and better immediately write to the mail [alex@alexgyver.ru] (mailto: alex@alexgyver.ru)
The library is open for refinement and your ** pull Request ** 'ow!

When reporting about bugs or incorrect work of the library, it is necessary to indicate:
- The version of the library
- What is MK used
- SDK version (for ESP)
- version of Arduino ide
- whether the built -in examples work correctly, in which the functions and designs are used, leading to a bug in your code
- what code has been loaded, what work was expected from it and how it works in reality
- Ideally, attach the minimum code in which the bug is observed.Not a canvas of a thousand lines, but a minimum code
