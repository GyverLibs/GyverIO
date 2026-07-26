#pragma once
#include <Arduino.h>

#include "../pin/PinIO.h"

#ifndef SOFTWIRE_DELAY_US
#define SOFTWIRE_DELAY_US 0
#endif

#ifndef SOFTWIRE_TIMEOUT_US
#define SOFTWIRE_TIMEOUT_US 1000
#endif

class SoftWire {
   public:
    void begin(uint8_t sda = SDA, uint8_t scl = SCL) {
        _sda.init(sda, INPUT_PULLUP);
        _scl.init(scl, INPUT_PULLUP);
        _active = true;
        _recover();
    }

    void setClock(uint32_t) {
        // uint32_t half = 500000UL / clock;
        // _delayUs = (half > 65535UL) ? 65535 : half;
    }

    void end() {
        if (_active) {
            _stop();
            _release(_sda);
            _release(_scl);
        }

        _active = false;
        _started = false;
        _rxLeft = 0;
    }

    void beginTransmission(uint8_t address) {
        _txStatus = 0;
        _rxLeft = 0;
        if (!_active) {
            _txStatus = 4;
            return;
        }
        if (!_start()) {
            _txStatus = 4;
            return;
        }
        if (!_writeByte((address << 1) | 0)) _txStatus = 2;
    }

    uint8_t endTransmission(bool stop = true) {
        if (stop && !_stop() && !_txStatus) _txStatus = 4;
        uint8_t status = _txStatus;
        _txStatus = 0;
        return status;
    }

    size_t write(uint8_t data) {
        if (_txStatus) return 0;
        if (!_writeByte(data)) {
            _txStatus = 3;
            return 0;
        }
        return 1;
    }

    size_t write(const uint8_t* data, size_t len) {
        size_t written = 0;
        while (written < len && write(data[written])) written++;
        return written;
    }

    uint8_t requestFrom(uint8_t address, uint8_t len) {
        _rxLeft = 0;
        if (!_active || !len) return 0;

        if (!_start()) return 0;
        if (!_writeByte((address << 1) | 1)) {
            _stop();
            return 0;
        }
        _rxLeft = len;
        return len;
    }

    uint8_t available() {
        return _rxLeft;
    }

    int read() {
        if (!_rxLeft) return -1;

        uint8_t data;
        if (!_readByte(data)) {
            _rxLeft = 0;
            _stop();
            return -1;
        }
        _rxLeft--;
        if (!_writeAck(_rxLeft)) {
            _rxLeft = 0;
            _stop();
            return -1;
        }
        if (!_rxLeft) _stop();
        return data;
    }

    size_t read(uint8_t* data, size_t len) {
        size_t res = 0;
        while (len--) {
            int r = read();
            if (r < 0) return res;
            *data++ = r;
            res++;
        }
        return res;
    }

   private:
    gio::PinIO _sda;
    gio::PinIO _scl;
    uint8_t _rxLeft = 0;
    uint8_t _txStatus = 0;
    bool _active = false;
    bool _started = false;

    void _delay() {
#if SOFTWIRE_DELAY_US > 0
        delayMicroseconds(SOFTWIRE_DELAY_US);
#endif
    }

    void _release(gio::PinIO& pin) {
        pin.mode(INPUT_PULLUP);
    }

    void _low(gio::PinIO& pin) {
        pin.low();
        pin.mode(OUTPUT);
    }

    bool _read(gio::PinIO& pin) {
        return pin.read();
    }

    bool _clockHigh() {
        _release(_scl);
        if (!_read(_scl)) {
            uint32_t start = micros();
            while (!_read(_scl)) {
                if (micros() - start >= SOFTWIRE_TIMEOUT_US) return false;
            }
        }
        _delay();
        return true;
    }

    void _clockLow() {
        _low(_scl);
        _delay();
    }

    bool _start() {
        if (_started) {
            _release(_sda);
            if (!_clockHigh()) return false;
        }
        _low(_sda);
        _delay();
        _clockLow();
        _started = true;
        return true;
    }

    bool _stop() {
        if (!_started) return true;
        _low(_sda);
        _delay();
        bool ok = _clockHigh();
        _release(_sda);
        _delay();
        _started = false;
        return ok && _read(_sda);
    }

    bool _writeBit(bool bit) {
        if (bit) _release(_sda);
        else _low(_sda);

        bool ok = _clockHigh();
        _clockLow();
        return ok;
    }

    bool _readBit(bool& bit) {
        _release(_sda);
        if (!_clockHigh()) return false;
        bit = _read(_sda);
        _clockLow();
        return true;
    }

    bool _writeByte(uint8_t data) {
        for (uint8_t mask = 0x80; mask; mask >>= 1) {
            if (!_writeBit(data & mask)) return false;
        }

        bool nack;
        return _readBit(nack) && !nack;
    }

    bool _readByte(uint8_t& data) {
        data = 0;
        for (uint8_t i = 0; i < 8; i++) {
            bool bit;
            if (!_readBit(bit)) return false;
            data <<= 1;
            if (bit) data |= 1;
        }
        return true;
    }

    bool _writeAck(bool ack) {
        return _writeBit(!ack);
    }

    void _recover() {
        _release(_sda);
        _release(_scl);
        _delay();

        if (_read(_sda)) return;

        for (uint8_t i = 0; i < 9 && !_read(_sda); i++) {
            _clockLow();
            _clockHigh();
        }
        _started = true;
        _stop();
    }
};
