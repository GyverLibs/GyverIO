#pragma once

#include <Arduino.h>

#include "../gio/gio.h"

// #define GIO_NO_MASK

namespace gio {

class PinIO {
   public:
    PinIO() {}
    PinIO(uint8_t pin, uint8_t mode = INPUT) {
        init(pin, mode);
    }

    void init(uint8_t pin, uint8_t mode = INPUT) {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        _out = portOutputRegister(digitalPinToPort(pin));
        _inp = portInputRegister(digitalPinToPort(pin));
        _mode = portModeRegister(digitalPinToPort(pin));
        _mask = digitalPinToBitMask(pin);
        this->mode(mode);
#else
        gio::init(pin, mode);
        _pin = pin;
#endif
    }

    void mode(uint8_t val) {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        switch (val) {
            case INPUT:
                greg_clr(_mode, _mask);
                greg_clr(_out, _mask);
                break;
            case INPUT_PULLUP:
                greg_clr(_mode, _mask);
                greg_set(_out, _mask);
                break;
            case OUTPUT:
                greg_set(_mode, _mask);
                break;
        }
#else
        gio::mode(_pin, val);
#endif
    }

    void write(uint8_t val) {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        greg_write(_out, _mask, val);
#else
        gio::write(_pin, val);
#endif
    }

    void high() {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        greg_set(_out, _mask);
#else
        gio::high(_pin);
#endif
    }

    void low() {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        greg_clr(_out, _mask);
#else
        gio::low(_pin);
#endif
    }

    void toggle() {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        *_inp = _mask;
#else
        gio::toggle(_pin);
#endif
    }

    int read() const {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        return greg_read(_inp, _mask);
#else
        return gio::read(_pin);
#endif
    }

    bool valid() const {
#if defined(__AVR__) && !defined(GIO_NO_MASK)
        return _mode;
#else
        return _pin != 0xff;
#endif
    }

    operator bool() const {
        return valid();
    }

   private:
#if defined(__AVR__) && !defined(GIO_NO_MASK)
    volatile uint8_t* _mode = nullptr;
    volatile uint8_t* _out = nullptr;
    volatile uint8_t* _inp = nullptr;
    uint8_t _mask = 0;
#else
    uint8_t _pin = 0xff;
#endif
};

}  // namespace gio