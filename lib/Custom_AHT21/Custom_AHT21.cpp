#include "Custom_AHT21.h"

bool CustomAHT21::begin() {
    if (!aht.begin()) {
        while (1)
            delay(10);
    }
    return true;
}

AHT21Data CustomAHT21::read() {
    AHT21Data data = {0, 0};
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    data.temp = temp.temperature;
    data.humidity = humidity.relative_humidity;

    return data;
}
