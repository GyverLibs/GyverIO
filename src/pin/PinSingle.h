#pragma once

#include <Arduino.h>

#include "../gio/gio.h"

// #define GIO_NO_MASK

namespace gio {

// single mode pin (output/input)
class PinSingle {
   public:
    PinSingle() {}
    PinSingle(uint8_t pin, uint8_t mode = INPUT) {
        init(pin, mode);
    }

    void init(uint8_t pin, uint8_t mode = INPUT) {
        gio::init(pin, mode);
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        _mask = digitalPinToBitMask(pin);
        _reg = (mode == OUTPUT)
                   ? portOutputRegister(digitalPinToPort(pin))
                   : portInputRegister(digitalPinToPort(pin));
#else
        _pin = pin;
#endif
    }

    void write(uint8_t val) {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        greg_write(_reg, _mask, val);
#else
        gio::write(_pin, val);
#endif
    }

    void high() {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        greg_set(_reg, _mask);
#else
        gio::high(_pin);
#endif
    }

    void low() {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        greg_clr(_reg, _mask);
#else
        gio::low(_pin);
#endif
    }

    void toggle() {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        *_reg ^= _mask;
#else
        gio::toggle(_pin);
#endif
    }

    int read() const {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        return greg_read(_reg, _mask);
#else
        return gio::read(_pin);
#endif
    }

    bool valid() const {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        return _reg;
#else
        return _pin != 0xff;
#endif
    }

    operator bool() const {
        return valid();
    }

   private:
#if defined(__AVR__) && !defined(GIO_NO_MASK)
    volatile uint8_t* _reg = nullptr;
    uint8_t _mask = 0;
#else
    uint8_t _pin = 0xff;
#endif
};

}  // namespace gio