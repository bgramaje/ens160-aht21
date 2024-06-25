#ifndef ENS16X_I2C_INTERFACE_H
#define ENS16X_I2C_INTERFACE_H

#include <Wire.h>
#include <ens16x.h>

using namespace ScioSense;

class I2cInterface : public virtual Utils::IoInterface<ENS16x::RegisterAddress, ENS16x::Result>
{
public:
    I2cInterface();
public:
    void begin(TwoWire& twoWire = Wire, uint8_t address = 0x52);
public:
    virtual ENS16x::Result read(const ENS16x::RegisterAddress& address, uint8_t* data, const size_t& size);
    virtual ENS16x::Result write(const ENS16x::RegisterAddress& address, uint8_t* data, const size_t& size);
private:
    uint8_t slaveAddress;
    TwoWire* wire;
};

I2cInterface::I2cInterface()
{
    wire                = nullptr;
    slaveAddress        = 0x52;
}

void I2cInterface::begin(TwoWire& twoWire, uint8_t address)
{
    wire         = &twoWire;
    slaveAddress = address;
}

ENS16x::Result I2cInterface::read(const ENS16x::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    ENS16x::Result result = ENS16x::Result::IOError;

    if (size == 0)
    {
        return result;
    }

    wire->beginTransmission(slaveAddress);
    wire->write((uint8_t)address);

    if (wire->endTransmission() == 0) // 0 == success
    {
        constexpr uint8_t MAX_CHUNK_SIZE = 32;

        size_t len = 0;
        while (len < size)
        {
            size_t bytesToRequest = ( MAX_CHUNK_SIZE < (size - len) ? MAX_CHUNK_SIZE : (size - len) );
            wire->requestFrom(slaveAddress, bytesToRequest);
            size_t n = wire->readBytes(data + len, bytesToRequest);
            len += n;

            if (n == 0)
            {
                break;
            }
        }

        if (len == size)
        {
            result = ENS16x::Result::Ok;
        }
    }

    return result;
}

ENS16x::Result I2cInterface::write(const ENS16x::RegisterAddress& address, uint8_t* data, const size_t& size)
{
    ENS16x::Result result = ENS16x::Result::IOError;

    wire->beginTransmission(slaveAddress);
    wire->write((uint8_t)address);
    wire->write(data, size);
    if (wire->endTransmission() == 0) // 0 == success
    {
        result = ENS16x::Result::Ok;
    }

    return result;
}

#endif //ENS16X_I2C_INTERFACE_H