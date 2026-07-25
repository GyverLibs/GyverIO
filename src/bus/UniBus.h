#pragma once
#include <stdint.h>

template <class Derived>
class UniBus {
   protected:
    // TRANSACTION
    // начать чтение
    bool beginRead(uint8_t reg, uint8_t len = 1) {
        return _bus()._busBeginRead(reg, len);
    }

    // начать запись
    bool beginWrite(uint8_t reg) {
        return _bus()._busBeginWrite(reg);
    }

    // закончить чтение/запись
    bool end() {
        return _bus()._busEnd();
    }

    // READ RAW
    bool read(uint8_t& value) {
        return _bus()._busRead(value);
    }

    bool read(uint8_t* data, uint8_t len) {
        while (len--) {
            if (!read(*data++)) return false;
        }
        return true;
    }

    bool readLE16(uint16_t& value) {
        uint8_t lsb, msb;
        if (!read(lsb) || !read(msb)) return false;
        value = uint16_t(lsb) | (uint16_t(msb) << 8);
        return true;
    }

    bool readBE16(uint16_t& value) {
        uint8_t lsb, msb;
        if (!read(msb) || !read(lsb)) return false;
        value = (uint16_t(msb) << 8) | uint16_t(lsb);
        return true;
    }

    // WRITE RAW
    bool write(uint8_t data) {
        return _bus()._busWrite(data);
    }

    bool write(const uint8_t* data, uint8_t len) {
        while (len--) {
            if (!write(*data++)) return false;
        }
        return true;
    }

    bool writeLE16(uint16_t value) {
        return write(value) && write(value >> 8);
    }

    bool writeBE16(uint16_t value) {
        return write(value >> 8) && write(value);
    }

    // REGISTER READ
    bool readReg(uint8_t reg, uint8_t& value) {
        return beginRead(reg, 1) && _finish(read(value));
    }

    bool readRegs(uint8_t reg, uint8_t* data, uint8_t len) {
        return beginRead(reg, len) && _finish(read(data, len));
    }

    bool readRegLE16(uint8_t reg, uint16_t& value) {
        return beginRead(reg, 2) && _finish(readLE16(value));
    }

    bool readRegBE16(uint8_t reg, uint16_t& value) {
        return beginRead(reg, 2) && _finish(readBE16(value));
    }

    // REGISTER WRITE
    bool writeReg(uint8_t reg, uint8_t value) {
        return writeRegs(reg, &value, 1);
    }

    bool writeReg(uint8_t reg, uint8_t value1, uint8_t value2) {
        return beginWrite(reg) && _finish(write(value1) && write(value2));
    }

    bool writeRegs(uint8_t reg, const uint8_t* data, uint8_t len) {
        return beginWrite(reg) && _finish(write(data, len));
    }

    bool writeRegLE16(uint8_t reg, uint16_t value) {
        return beginWrite(reg) && _finish(writeLE16(value));
    }

    bool writeRegBE16(uint8_t reg, uint16_t value) {
        return beginWrite(reg) && _finish(writeBE16(value));
    }

   private:
    bool _finish(bool ok) {
        return end() && ok;
    }
    Derived& _bus() {
        return static_cast<Derived&>(*this);
    }
};
