#pragma once
#include <Arduino.h>

#include "../pin/PinSingle.h"

#ifndef SOFTSPI_DELAY_US
#define SOFTSPI_DELAY_US 1
#endif

class SoftSPI {
   public:
    void begin(uint8_t miso, uint8_t mosi, uint8_t clk) {
        if (miso != 0xff) {
            _miso.init(miso, INPUT);
        }

        if (mosi != 0xff) {
            _mosi.init(mosi, OUTPUT);
            _mosi.low();
        }

        _clk.init(clk, OUTPUT);
        _clk.low();
    }

    void beginTX(uint8_t mosi, uint8_t clk) {
        begin(0xff, mosi, clk);
    }

    void beginRX(uint8_t miso, uint8_t clk) {
        begin(miso, 0xff, clk);
    }

    // todo
    // void end() {}

    void setClock(uint32_t clock) {
        (void)clock;
        // if (!clock) {
        //     _delayUs = SOFTSPI_DELAY_US;
        //     return;
        // }

        // uint32_t half = 500000UL / clock;
        // _delayUs = (half > 65535UL) ? 65535 : half;
    }

    size_t write(uint8_t data) {
        if (!_mosi) return 0;

        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            if (data & mask) _mosi.high();
            else _mosi.low();

            _delay();
            _clk.high();
            _delay();
            _clk.low();
        }

        return 1;
    }

    size_t write(const uint8_t* data, size_t len) {
        size_t i = len;
        while (i--) write(*data++);
        return len;
    }

    int read() {
        if (!_miso) return -1;

        if (_mosi) _mosi.high();  // dummy 0xff
        uint8_t res = 0;

        for (uint8_t i = 8; i; i--) {
            _delay();
            _clk.high();
            _delay();

            res <<= 1;
            if (_miso.read()) res |= 1;
            _clk.low();
        }

        return res;
    }

    size_t read(uint8_t* data, size_t len) {
        size_t i = len;
        while (i--) *data++ = read();
        return len;
    }

    uint8_t transfer(uint8_t data) {
        if (!_mosi) return 0;

        if (!_miso) {
            write(data);
            return 0;
        }

        uint8_t res = 0;

        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            if (data & mask) _mosi.high();
            else _mosi.low();

            _delay();
            _clk.high();
            _delay();

            res <<= 1;
            if (_miso.read()) res |= 1;
            _clk.low();
        }

        return res;
    }

    void transfer(uint8_t* data, size_t len) {
        while (len--) {
            *data = transfer(*data);
            data++;
        }
    }

   private:
    gio::PinSingle _miso;
    gio::PinSingle _mosi;
    gio::PinSingle _clk;
    // uint16_t _delayUs = SOFTSPI_DELAY_US;

    void _delay() {
        // if (_delayUs) delayMicroseconds(_delayUs);
    }
};
