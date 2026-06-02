This is an automatic translation and may be incorrect in some places. See the source README and examples for authoritative information.

[![latest](https://img.shields.io/github/v/release/GyverLibs/GyverIO.svg?color=brightgreen)](https://github.com/GyverLibs/GyverIO/releases/latest/download/GyverIO.zip)
[![PIO](https://badges.registry.platformio.org/packages/gyverlibs/library/GyverIO.svg)](https://registry.platformio.org/libraries/gyverlibs/GyverIO)
[![Foo](https://img.shields.io/badge/Website-AlexGyver.ru-blue.svg?style=flat-square)](https://alexgyver.ru/)
[![Foo](https://img.shields.io/badge/%E2%82%BD%24%E2%82%AC%20%D0%9F%D0%BE%D0%B4%D0%B4%D0%B5%D1%80%D0%B6%D0%B0%D1%82%D1%8C-%D0%B0%D0%B2%D1%82%D0%BE%D1%80%D0%B0-orange.svg?style=flat-square)](https://alexgyver.ru/support_alex/)
[![Foo](https://img.shields.io/badge/README-ENGLISH-blueviolet.svg?style=flat-square)](https://github-com.translate.goog/GyverLibs/GyverIO?_x_tr_sl=ru&_x_tr_tl=en)  

[![Foo](https://img.shields.io/badge/ПОДПИСАТЬСЯ-НА%20ОБНОВЛЕНИЯ-brightgreen.svg?style=social&logo=telegram&color=blue)](https://t.me/GyverLibs)

# GyverIO
Fast features for working with AVR pins (see full list in gio avr.h), ESP8266, ESP32
- Acceleration by an average of 20-30 times, the final time for all architectures is almost the same.
- Classes for quick pin control
- Separate treatment of constant and nonconstant pin cases for AVR
- Quick implementation of shiftIn/shiftOut functions
- Universal hard SPI/soft SPI class for use in libraries

## Speed.
### Conditions of measurement
| | Version | Frequency |
|---------|--------|---------|
| AVR     | 1.8.19 | 16      |
| ESP8266 | 3.1.2  | 80      |
| ESP32   | 2.0.11 | 240     |
| ESP32C3 | 2.0.11 | 80      |

### GPIO (us)
|              | write NC | write       | read NC   | read        | mode NC | mode        |
|--------------|----------|-------------|-----------|-------------|---------|-------------|
| AVR Ardu     | **5.3**  | 5.3         | **4.8**   | 4.8         | **3.3** | 3.3         |
| AVR gio      | 1.6      | ***0.125*** | 1.75      | ***0.075*** | 1.8     | ***0.125*** |
|              |          |             |           |             |         |             |
| ESP8266 Ardu | **1.5**  | 1.5         | **0.54**  | 0.54        | **1.4** | 1.4         |
| ESP8266 gio  | 0.29     | ***0.08***  | 0.5       | ***0.17***  | 1.29    | ***0.58***  |
|              |          |             |           |             |         |             |
| ESP32 Ardu   | **0.33** | 0.33        | **0.124** | 0.124       | **16**  | 16          |
| ESP32 gio    | 0.04     | ***0.04***  | 0.085     | ***0.085*** | 0.126   | ***0.08***  |
|              |          |             |           |             |         |             |
| ESP32C3 Ardu | **0.91** | 0.91        | **0.25**  | 0.25        | **21**  | 21          |
| ESP32C3 gio  | 0.05     | ***0.05***  | 0.4       | ***0.08***  | 0.49    | ***0.08***  |

> *NC* - pins are not constants

> **Fat** gets the worst time (Arduino not constants), ***Fat cursive*** gets the best time (gio constants)

### Shift (MHz)
|         | shiftOut | gio::shift |
|---------|----------|------------|
| AVR NC  | 0.06     | 0.66       |
| AVR     | 0.06     | 1.3        |
| ESP8266 | 0.2      | 1.1        |
| ESP32   | 0.96     | 6          |
| ESP32C3 | 0.35     | 2.6        |

> *NC* - pins are not constants

### Compatibility
Compatible with all Arduino platforms (Arduino features are used)
- For esp8266 and esp32, fast`pinMode()` (`mode()`) works only on regimes`INPUT`/`OUTPUT`! In other modes, the staff is called`pinMode()`

## Contents
- [Documentation.](#docs)
- [Use of use](#usage)
- [Versions](#versions)
- [Installation](#install)
- [Bugs and feedback](#feedback)

<a id="docs"></a>
## Documentation.
### gio
Fast functions for working with pins

```cpp
int gio::read(int P);
void gio::high(int P);
void gio::low(int P)
void gio::write(int P, int V);

//
void gio::toggle(int P);

// Pin mode. For esp8266/esp32 only INPUT/OUTPUT!
void gio::mode(int P, int V);

// call esp8266/esp32 when initializing a pin
// Otherwise, mode() will not work.
void gio::init(int P);
```
> Esp8266/esp32 must be called`gio::init`before use!

### gio::shift
Fast analogue shiftIn/shiftOut (sending data with a cloke)

```cpp
// read the package. Return true if at least one bit is different
bool gio::shift::read(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// byte
uint8_t gio::shift::read_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t delay = 0);

// Read the package + cs pin. Return true if at least one bit is different
bool gio::shift::read_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// read byte + cs pin
uint8_t gio::shift::read_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t delay = 0);

// packet
void gio::shift::send(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// byte
void gio::shift::send_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t data, uint8_t delay = 0);

// send a packet + cs pin
void gio::shift::send_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// send byte + cs pin
void gio::shift::send_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t data, uint8_t delay = 0);
```

Parameter`order`Maybe:
- `LSBFIRST`/`LSB_NORMAL`- LSB, direct order of bytes
- `MSBFIRST`/`MSB_NORMAL`- MSB, direct byte order.
- `LSB_REVERSE`- LSB, reverse byte order.
- `MSB_REVERSE`- MSB, reverse byte order.

#### Note
- `delay`in microseconds, reduces the transmission speed. For example,`1`μx will limit the speed to ~1 MHz, 2 μs to ~500 kHz
- Pins need to be configured as`OUTPUT`independently before sending (when starting the program for example)

### gio::SSPI
Universal class of software and hardware SPI with optimization of the number of variables for pins

```cpp
SSPI<0, freq> spi;                  // pinless
SSPI<0, freq, cs> spi;              // hardware-pinned CS in a template
SSPI<0, freq> spi(cs);              // hardware-pinned CS in class
SSPI<1, freq, cs, dt, clk> spi;	    // patterned
SSPI<1, freq> spi(cs, dt, clk);	    // pin-in-class
```

### Compilation settings
```cpp
#define GIO_USE_ARDUINO     // Disable fast functions (replace with standard ones)
#define GIO_NO_MASK         // disable fast mask-based IO for AVR (in PinIO class and all shift)
```

<a id="usage"></a>
## Use of use

```cpp
#include <GyverIO_SPI.h>

gio::write(3, 1);   // pin 3

// send data to pins 3 and 4
uint8_t data[] = {34, 63, 231, 9};
gio::shift::send(3, 4, MSBFIRST, data, 4);

SSPI<0, f, cs> spi;
spi.send(0x12);
```

<a id="versions"></a>
## Versions
- v1.0
- v1.1 - 3 times accelerated AVR non-const, updated tables
- v1.2 - corrected error!
- v1.2.1 - small optimization
- v1.2.2 - Inversion added to shift
- v1.2.4 - fix bug in gio::shift::read for AVR NC
- v1.2.5 - True return added to gio::shift::read when buffer changes
- v1.3.0 - AVR/mode/NC critical error fixed
- v1.3.1 - added an additional delay to the shift for symmetry of the block
- v1.3.2 - SSPI is included in a separate file so as not to interfere with compilation on some platforms
- v1.3.4 - ESP32C6 support

<a id="install"></a>
## Installation
- The library can be found under the name **GyverIO** and installed through the library manager in:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Download the library](https://github.com/GyverLibs/GyverIO/archive/refs/heads/main.zip).zip archive for manual installation:
    - Unpack and put in *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Unpack and put in *C:\Program Files\Arduino\libraries* (Windows x32)
    - Unpack and put in *Documents/Arduino/libraries/ *
    - (Arduino IDE) Automatic installation from .zip: *Sketch/Connect library/Add .ZIP library...* and specify downloaded archive
- Read more detailed instructions for installing libraries[here](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Update
- I recommend always updating the library: new versions fix errors and bugs, as well as optimize and add new features.
- Through the library manager IDE: find the library as when installing and click "Update"
- Manually: **Delete the folder with the old version** and then put the new one in its place. “Replacement” can not be done: sometimes new versions delete files that will remain when replaced and can lead to errors!

<a id="feedback"></a>
## Bugs and feedback
If you find bugs, create **Issue**, or better write to the mail immediately.[alex@alexgyver.ru](mailto:alex@alexgyver.ru)  
The library is open for revision and your **Pull Requests*!

When reporting bugs or incorrect work of the library, it is necessary to specify:
- Library version
- What is used by the IC
- SDK version (for ESP)
- Arduino IDE version
- Are embedded examples that use features and designs that cause bugs in your code working correctly?
- What code was downloaded, what work was expected from it and how it works in reality
- Ideally, attach the minimum code in which the bug is observed. Not a canvas of a thousand lines, but a minimum code.
