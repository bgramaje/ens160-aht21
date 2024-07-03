#ifndef Custom_AHT21
#define Custom_AHT21

#include <Arduino.h>
#include <Adafruit_AHTX0.h>

/**
 * @brief struct for data incoming from AHT21 sensor
 *
 * It serves as schema for storing intro friendly variables the attributes
 * received by the AHT21 sensor reading
 */
struct AHT21Data
{
    float temp;
    float humidity;
};

class CustomAHT21
{
public:
    bool begin();
    AHT21Data read();

private:
    Adafruit_AHTX0 aht;
};

#endif
