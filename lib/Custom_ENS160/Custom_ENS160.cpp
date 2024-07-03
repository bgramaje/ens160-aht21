#include "Custom_ENS160.h"

using namespace ScioSense;

void CustomENS160::begin(I2cInterface* i2c)
{
    while (!ens.begin(i2c))
    {
        delay(1000);
    }
    ens.startStandardMeasure();
}

ENS160Data CustomENS160::read()
{
    ENS160Data data = {0, 0};

    if (ens.update() != ENS16x::Result::Ok)
    {
        return data;
    }

    if (hasFlag(ens.getDeviceStatus(), ENS16x::DeviceStatus::NewData))
    {
        data.tvoc = ens.getTvoc();
        data.eco2 = ens.getEco2();
    }

    return data;
}
