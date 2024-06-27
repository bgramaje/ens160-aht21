#ifndef Custom_ENS160_H
#define Custom_ENS160_H

#include <Arduino.h>
#include <ScioSense_ENS16x.h>
#include "ens16x_i2c_interface.h"

#define ENS160_I2C_ADDRESS 0x53

using namespace ScioSense;

/**
 * @brief struct for data incoming from ENS160 sensor
 *
 * It serves as schema for storing into friendly variables the attributes
 * received by the ENS160 sensor reading
 */
struct ENS160Data
{
    float tvoc;
    float eco2;
};

class ENS160Sensor
{
public:
    ENS160 ens160;
    I2cInterface i2c;
    void begin();
    ENS160Data read();
};

#endif // Custom_ENS160_H
