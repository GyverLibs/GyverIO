#pragma once
#include "UniBus.h"

template <class WireClassT>
class WireBus : public UniBus<WireBus<WireClassT>> {
    friend class UniBus<WireBus<WireClassT>>;

   public:
    explicit WireBus(WireClassT& wire, uint8_t address)
        : _wire(wire), _address(address), _reading(false) {}

    // получить адрес
    uint8_t getAddress() const {
        return _address;
    }

    // установить адрес
    void setAddress(uint8_t address) {
        _address = address;
    }

    // проверить есть ли датчик на линии по адресу
    bool ping() {
        _wire.beginTransmission(_address);
        return _wire.endTransmission(true) == 0;
    }

   private:
    WireClassT& _wire;
    uint8_t _address;
    bool _reading;

    bool _busBeginRead(uint8_t reg, uint8_t len) {
        _wire.beginTransmission(_address);
        _wire.write(reg);  // todo check
        if (_wire.endTransmission(false) != 0) return false;

        _reading = _wire.requestFrom(_address, len) == len;
        return _reading;
    }

    bool _busRead(uint8_t& value) {
        int r = _wire.read();
        if (r < 0) return false;
        value = uint8_t(r);
        return true;
    }

    bool _busBeginWrite(uint8_t reg) {
        _wire.beginTransmission(_address);
        return _wire.write(reg);  // todo check
    }

    bool _busWrite(uint8_t data) {
        return _wire.write(data) == 1;
    }

    bool _busEnd() {
        if (_reading) {
            _reading = false;
            return true;
        }
        return _wire.endTransmission(true) == 0;
    }
};
