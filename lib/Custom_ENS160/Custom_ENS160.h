#ifndef Custom_ENS160_H
#define Custom_ENS160_H

#include <Arduino.h>

#include "ens16x.h"
#include "ens160.h"
#include "ens16x_i2c_interface.h"

#define ENS160_I2C_ADDRESS 0x53


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

class CustomENS160
{
public:
    void begin(I2cInterface* i2c);
    ENS160Data read();
    ENS160 ens;  // Declare ens160 as a private member
};

#endif // Custom_ENS160_H
