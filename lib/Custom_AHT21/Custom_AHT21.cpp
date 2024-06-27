#include "Custom_AHT21.h"

bool AHT21Sensor::begin() {
    if (!aht.begin()) {
        while (1)
            delay(10);
    }
    return true;
}

AHT21Data AHT21Sensor::read() {
    AHT21Data data = {0, 0};
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    data.temp = temp.temperature;
    data.humidity = humidity.relative_humidity;

    return data;
}
