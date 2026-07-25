#pragma once
#include <Arduino.h>
#include <SPI.h>

#include "../pin/PinSingle.h"

#ifndef SOFTSPI_DELAY_US
#define SOFTSPI_DELAY_US 0
#endif

// SoftSPIBase
template <typename Derived>
class SoftSPIBase {
   public:
    void end() {}
    void beginTransaction(SPISettings) {}
    void endTransaction() {}

    // MSB first
    uint16_t transfer16(uint16_t data) {
        return (uint16_t(_derived().transfer(uint8_t(data >> 8))) << 8) |
               uint16_t(_derived().transfer(uint8_t(data)));
    }

    // передаётся и заменяется принятыми данными
    void transfer(uint8_t* data, size_t len) {
        while (len--) {
            *data = _derived().transfer(*data);
            ++data;
        }
    }

   protected:
    void _delay() {
#if SOFTSPI_DELAY_US > 0
        delayMicroseconds(SOFTSPI_DELAY_US);
#endif
    }

   private:
    Derived& _derived() {
        return static_cast<Derived&>(*this);
    }
};

// SoftSPI
class SoftSPI : public SoftSPIBase<SoftSPI> {
   public:
    using Base = SoftSPIBase<SoftSPI>;
    using Base::transfer;

    void begin(uint8_t miso, uint8_t mosi, uint8_t clk) {
        _miso.init(miso, INPUT);

        _mosi.init(mosi, OUTPUT);
        _mosi.low();

        _clk.init(clk, OUTPUT);
        _clk.low();
    }

    uint8_t transfer(uint8_t data) {
        uint8_t res = 0;

        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            (data & mask) ? _mosi.high() : _mosi.low();

            _delay();
            _clk.high();
            _delay();

            res <<= 1;
            if (_miso.read()) res |= 1;
            _clk.low();
        }

        return res;
    }

   private:
    gio::PinSingle _miso;
    gio::PinSingle _mosi;
    gio::PinSingle _clk;
};

// SoftSPI_TX
class SoftSPI_TX : public SoftSPIBase<SoftSPI_TX> {
   public:
    using Base = SoftSPIBase<SoftSPI_TX>;
    using Base::transfer;

    void begin(uint8_t mosi, uint8_t clk) {
        _mosi.init(mosi, OUTPUT);
        _mosi.low();

        _clk.init(clk, OUTPUT);
        _clk.low();
    }

    uint8_t transfer(uint8_t data) {
        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            (data & mask) ? _mosi.high() : _mosi.low();

            _delay();
            _clk.high();
            _delay();
            _clk.low();
        }

        return 0;
    }

   private:
    gio::PinSingle _mosi;
    gio::PinSingle _clk;
};

// SoftSPI_RX
class SoftSPI_RX : public SoftSPIBase<SoftSPI_RX> {
   public:
    using Base = SoftSPIBase<SoftSPI_RX>;
    using Base::transfer;

    void begin(uint8_t miso, uint8_t clk) {
        _miso.init(miso, INPUT);

        _clk.init(clk, OUTPUT);
        _clk.low();
    }

    uint8_t transfer(uint8_t) {
        uint8_t res = 0;

        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            _delay();
            _clk.high();
            _delay();

            res <<= 1;
            if (_miso.read()) res |= 1;
            _clk.low();
        }

        return res;
    }

   private:
    gio::PinSingle _miso;
    gio::PinSingle _clk;
};
