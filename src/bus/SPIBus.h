#pragma once
#include <SPI.h>

#include "../gio/gio.h"
#include "UniBus.h"

template <class SPIClassT>
class SPIBus : public UniBus<SPIBus<SPIClassT>> {
    friend class UniBus<SPIBus<SPIClassT>>;

   public:
    explicit SPIBus(SPIClassT& spi, uint8_t cs, uint32_t frequency = 4000000UL, uint8_t bitOrder = MSBFIRST, uint8_t dataMode = SPI_MODE0)
        : _spi(spi), _settings(frequency, bitOrder, dataMode), _cs(cs) {}

    bool begin() {
        gio::init(_cs, OUTPUT);
        gio::high(_cs);
        return true;
    }

   private:
    SPIClassT& _spi;
    SPISettings _settings;
    uint8_t _cs;

    bool _busBeginRead(uint8_t reg, uint8_t) {
        return _begin(reg);
    }

    bool _busRead(uint8_t& value) {
        value = _spi.transfer(0xFF);
        return true;
    }

    bool _busBeginWrite(uint8_t reg) {
        return _begin(reg);
    }

    bool _begin(uint8_t reg) {
        _spi.beginTransaction(_settings);
        gio::low(_cs);
        _spi.transfer(reg);
        return true;
    }

    bool _busWrite(uint8_t data) {
        _spi.transfer(data);
        return true;
    }

    bool _busEnd() {
        gio::high(_cs);
        _spi.endTransaction();
        return true;
    }
};
