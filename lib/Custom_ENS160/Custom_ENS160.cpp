#include "Custom_ENS160.h"

void ENS160Sensor::begin()
{
    i2c.begin(Wire, ENS160_I2C_ADDRESS)
    while (!ens160.begin(&i2c))
    {
        delay(1000);
    }
    ens160.startStandardMeasure();
    return true;
}

ENS160Data ENS160Sensor::read()
{
    ENS160Data data = {0, 0};

    if (ens160.update() == ENS16x::Result::Ok)
    {
        if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewData))
        {
            data.tvoc = ens160.getTvoc();
            data.eco2 = ens160.getEco2();
        }
    }
    return data;
}
