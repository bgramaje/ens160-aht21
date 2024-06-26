#include <Arduino.h>
#include <ScioSense_ENS16x.h>
#include <Wire.h>
#include <Adafruit_AHTX0.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

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

// wifi options
const char *WIFI_SSID = "MiFibra-486C";
const char *WIFI_PASSWORD = "2p2gm2Ss";

const char *MQTT_HOST = "192.168.1.19";
const char *MQTT_USER = "test";
const char *MQTT_PASWORD = "test";

#define MQTT_PORT 1883
#define MQTT_TOPIC "sensors/"

// pin and I2C configurations
#define LED_PIN 2
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQUENCY 100000 // I2C frequency in Hz (100 kHz)
// ens160 i2c address
#define I2C_ADDRESS 0x53

I2cInterface i2c;
ENS160 ens160;
Adafruit_AHTX0 aht;

WiFiClient wifi;
PubSubClient client(wifi);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

struct ENS160_data
{
    float tvoc;
    float eco2;
};

struct AHT21_data
{
    float temp;
    float humidity;
};

String getFormattedTime(unsigned long epochTime) {
  time_t rawTime = epochTime;
  struct tm *timeInfo;
  timeInfo = gmtime(&rawTime); // Convert epoch time to UTC

  char buffer[25];
  strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeInfo); // Format time as ISO 8601

  return String(buffer);
}

void initializeWiFi()
{
    WiFi.mode(WIFI_STA);
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
    debugln(WIFI_SSID);
    debug("\tIP: ");
    debug(WiFi.localIP());
    debug("\tMAC: ");
    debugln(WiFi.macAddress());
}

void initializeMQTT()
{
    debug("[MQTT] Connecting to: ");
    debug(MQTT_HOST);
    debug(":");
    debug(MQTT_PORT);
    debugln();

    client.setServer(MQTT_HOST, MQTT_PORT);
    while (!client.connected())
    {
        if (client.connect("ESP32Client", MQTT_USER, MQTT_PASWORD))
        {
            debugln("[MQTT]: Successfully connected to broker");
            digitalWrite(LED_PIN, HIGH);
        }
        else
        {
            debug("Failed, rc=");
            debug(client.state());
            debugln(" Retrying in 5 seconds...");
            delay(2000);
        }
    }
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
    initializeMQTT();
    initializeI2C();
    initializeSensors();
    timeClient.begin();
    timeClient.setTimeOffset(0);
    debugln("Setup complete.");
}

ENS160_data readENS160()
{
    ENS160_data data;
    // ens160.wait();

    if (ens160.update() == ENS16x::Result::Ok)
    {
        if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewData))
        {
            debug("AQI UBA: ");
            debug((uint8_t)ens160.getAirQualityIndex_UBA());
            debug("\tTVOC: ");
            data.tvoc = ens160.getTvoc();
            debug(data.tvoc);
            debug("\tECO2: ");
            data.eco2 = ens160.getEco2();
            debugln(data.eco2);
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
    return data;
}

AHT21_data readAHT21()
{
    AHT21_data data;
    sensors_event_t humidity, temp;
    aht.getEvent(&humidity, &temp);

    data.temp = temp.temperature;
    data.humidity = humidity.relative_humidity;

    debug("TEMP: ");
    debug(data.temp);
    debug("\tHUM: ");
    debugln(data.humidity);
    return data;
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        initializeWiFi();
    }

    debug("[loop] WiFi");
    debug(WiFi.SSID());
    debug("\t RSSI: ");
    debugln(WiFi.RSSI());

    if (!client.connected())
    {
        digitalWrite(LED_PIN, LOW);
        initializeMQTT();
    }

    // call loop() regularly to maintain MQTT connection and handle incoming messages
    client.loop();

    // retrieve timestamp
    timeClient.update();
    unsigned long epochTime = timeClient.getEpochTime();
    String timestamp = getFormattedTime(timeClient.getEpochTime());

    ENS160_data ens160_data = readENS160();
    AHT21_data aht21_data = readAHT21();

    StaticJsonDocument<200> json;

    json["timestamp"] = timestamp;
    json["eco2"] = ens160_data.eco2;
    json["tvoc"] = ens160_data.tvoc;
    json["humidity"] = aht21_data.humidity;
    json["temperature"] = aht21_data.temp;

    char jsonBuffer[256];
    serializeJson(json, jsonBuffer);

    // Construct the topic with MAC address, removing colons
    String mac = String(WiFi.macAddress());
    mac.replace(":", "");
    String topic = String(MQTT_TOPIC) + mac;

    // Publish the JSON string to the constructed topic
    client.publish(topic.c_str(), jsonBuffer);

    debug("[loop] Published message to ");
    debugln(topic.c_str());

    delay(15000); // 15 seconds delay
}
