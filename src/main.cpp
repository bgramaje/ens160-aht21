#include <Arduino.h>
#include <ScioSense_ENS16x.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>

#include "ens16x_i2c_interface.h"
using namespace ScioSense;

// debug print
#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

// WiFi options
const char *WIFI_SSID = "MiFibra-486C";
const char *WIFI_PASSWORD = "2p2gm2Ss";

// pin and I2C configurations
#define LED_PIN 2
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQUENCY 100000 // I2C frequency in Hz (100 kHz)
#define I2C_ADDRESS 0x53

I2cInterface i2c;
ENS160 ens160;
Adafruit_AHTX0 aht;

void initializeWiFi()
{
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    debug("[WiFi] Connecting to: ");
    debug(WIFI_SSID);
    debug(" ");

    while (WiFi.status() != WL_CONNECTED)
    {
        debug(".");
        delay(500);
    }

    debugln();
    debug("[WiFi] Successfully connected to: ");
    debug(WIFI_SSID);
    debug("\tIP: ");
    debugln(WiFi.localIP());
}

void initializeDebugging()
{
    Serial.begin(115200);
    if (DEBUG == 1)
    {
        ens160.enableDebugging(Serial);
    }
}

void initializeI2C()
{
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
    i2c.begin(Wire, I2C_ADDRESS);
}

void initializeSensors()
{
    // initialize ENS160 sensor
    debugln("[initializeSensors] Initializing ENS160.");
    while (!ens160.begin(&i2c))
    {
        debug(".");
        delay(1000);
    }
    debugln("[initializeSensors] ENS160 initialized successfully.");
    ens160.startStandardMeasure();

    // initialize AHT21 sensor
    debug("[initializeSensors] Initializing AHT21.");
    if (!aht.begin())
    {
        debug(".");
        while (1)
            delay(10);
    }
    debugln("[initializeSensors] AHT21 initialized successfully.");
 }

void setup()
{
    debugln("Starting setup...");
    initializeDebugging();
    pinMode(LED_PIN, OUTPUT);
    initializeWiFi();
    initializeI2C();
    initializeSensors();
    debugln("Setup complete.");
}

void readENS160()
{
    ens160.wait();

    if (ens160.update() == ENS16x::Result::Ok)
    {
        if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewData))
        {
            debug("AQI UBA: ");
            debug((uint8_t)ens160.getAirQualityIndex_UBA());
            debug("\tTVOC: ");
            debug(ens160.getTvoc());
            debug("\tECO2: ");
            debugln(ens160.getEco2());
        }

        if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewGeneralPurposeData))
        {
            debug("RS0: ");
            debug(ens160.getRs0());
            debug("\tRS1: ");
            debug(ens160.getRs1());
            debug("\tRS2: ");
            debug(ens160.getRs2());
            debug("\tRS3: ");
            debugln(ens160.getRs3());
        }
    }
}

void readAHT21()
{
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    debug("TEMP: ");
    debug(temp.temperature);
    debug("\tHUM: ");
    debugln(humidity.relative_humidity);
}

void loop()
{
    digitalWrite(LED_PIN, HIGH);
    delay(2000);
    digitalWrite(LED_PIN, LOW);
    delay(2000);

    readENS160();
    readAHT21();

    delay(2000);
}
