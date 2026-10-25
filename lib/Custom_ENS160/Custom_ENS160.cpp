#include "Custom_ENS160.h"

using namespace ScioSense;

void CustomENS160::ensureStandardMode()
{
    // Mode changes are accepted from idle. Standard mode starts the heater;
    // without it the part stays in deep sleep / idle and both outputs stay 0.
    // ENS16x name for the suggested S_ENS160_OPMODE_STD register write.
    ens.setOperatingMode(ENS160::OperatingMode::Idle);
    delay(20);
    ens.setOperatingMode(ENS160::OperatingMode::Standard);
}

void CustomENS160::begin(I2cInterface* i2c)
{
    while (!ens.begin(i2c))
    {
        delay(1000);
    }

    // begin() only probes PART_ID and leaves the sensor idle. Enter standard
    // gas-sensing mode and retry the write if the bus rejects it.
    for (uint8_t attempt = 0; attempt < 5; attempt++)
    {
        if (ens.setOperatingMode(ENS160::OperatingMode::Standard) == ENS16x::Result::Ok)
        {
            return;
        }
        delay(1000);
    }

    ensureStandardMode();
}

ENS160Data CustomENS160::read()
{
    ENS160Data data = {0, 0};

    ENS16x::Result result = ens.update();
    if (result == ENS16x::Result::IOError)
    {
        return data;
    }

    ENS16x::DeviceStatus status = ens.getDeviceStatus();

    // STATAS clear means no operating mode is running, so the heater is off
    // and eCO2 / TVOC read as 0. Put the sensor back into standard mode.
    if (!hasFlag(status, ENS16x::DeviceStatus::Active))
    {
        ensureStandardMode();
        return data;
    }

    if (result == ENS16x::Result::Ok && hasFlag(status, ENS16x::DeviceStatus::NewData))
    {
        data.tvoc = ens.getTvoc();
        data.eco2 = ens.getEco2();
    }

    return data;
}
