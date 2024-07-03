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

#endif 