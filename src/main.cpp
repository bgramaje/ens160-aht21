#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <time.h>

#include "Custom_AHT21.h"
#include "Custom_ENS160.h"

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

// mqtt options
IPAddress MQTT_HOST(192, 168, 1, 19);

const char *MQTT_USER = "test";
const char *MQTT_PASWORD = "test";

#define MQTT_PORT 1883
#define MQTT_TOPIC "sensors/"

#define LED_PIN 2

// pin and I2C configurations
#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQUENCY 100000 // I2C frequency in Hz (100 kHz)


long lastMsg = 0;


I2cInterface i2c;

WiFiClient wifi;
PubSubClient client(wifi);

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 0, 60000);

CustomAHT21 aht21;
CustomENS160 ens160;

/**
 * @brief Retrieves MQTT topic given the MAC address of the device
 *
 * @returns the MQTT topic
 */
String getMqttTopic()
{
    String mac = String(WiFi.macAddress());
    mac.replace(":", "");
    return mac;
}

/**
 * @brief Transforms given time into format "%Y-%m-%dT%H:%M:%SZ"
 *
 * @param epochTime the current time
 * @returns the formmated time
 */
String getFormattedTime(unsigned long epochTime)
{
    time_t rawTime = epochTime;
    struct tm *timeInfo;
    timeInfo = gmtime(&rawTime); // Convert epoch time to UTC

    char buffer[25];
    strftime(buffer, sizeof(buffer), "%Y-%m-%dT%H:%M:%SZ", timeInfo); // Format time as ISO 8601

    return String(buffer);
}


/**
 * @brief Initializes the WiFi connection.
 *
 * This function attempts to connect to the specified WiFi network
 * using the provided SSID and password. It will block until the
 * connection is successful.
 */
void initializeWiFi()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    String debugLog = "[WiFi] Connecting to: " + String(WIFI_SSID) + " ";
    debug(debugLog);
    while (WiFi.status() != WL_CONNECTED)
    {
        debug(".");
        delay(500);
    }
    debugln();
    debugln("[WiFi] Successfully connected.");
}

void initializeMqtt()
{
    String debugLog = "[MQTT] Connecting to: " + MQTT_HOST.toString() + ": ";
    debug(debugLog.c_str());
    debugln();

    String mac = getMqttTopic();

    client.setKeepAlive(60);
    client.setServer(MQTT_HOST, MQTT_PORT);

    while (!client.connected())
    {
        if (client.connect(mac.c_str(), MQTT_USER, MQTT_PASWORD))
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
        ens160.ens.enableDebugging(Serial);
    }
}

void initializeI2C()
{
    Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
    i2c.begin(Wire, ENS160_I2C_ADDRESS);
}

void initializeSensors()
{
    // initialize ENS160 sensor
    debugln("[initializeSensors] Initializing ENS160.");
    ens160.begin(&i2c);
    debugln("[initializeSensors] ENS160 initialized successfully.");

    // initialize AHT21 sensor
    debug("[initializeSensors] Initializing AHT21.");
    aht21.begin();
    debugln("[initializeSensors] AHT21 initialized successfully.");
}

void setup()
{
    debugln("Starting setup...");
    pinMode(LED_PIN, OUTPUT);

    initializeDebugging();
    initializeWiFi();
    initializeMqtt();
    initializeI2C();
    initializeSensors();

    timeClient.begin();
    timeClient.setTimeOffset(0);
    debugln("Setup complete.");
}

void loop()
{
    if (WiFi.status() != WL_CONNECTED)
    {
        initializeWiFi();
    }

    if (!client.connected())
    {
        digitalWrite(LED_PIN, LOW);
        initializeMqtt();
    }

    long now = millis();
    if (now == 0 || now - lastMsg > 15000)
    {
        String wifiLog = "[loop] WiFi: " + WiFi.SSID() + "\tIP: " + (WiFi.localIP()).toString() + "\t RSSI: " + WiFi.RSSI();
        debugln(wifiLog);

        lastMsg = now;

        // retrieve timestamp
        timeClient.update();
        unsigned long epochTime = timeClient.getEpochTime();
        String timestamp = getFormattedTime(timeClient.getEpochTime());
        debugln(timestamp);

        AHT21Data aht21d = aht21.read();
        String aht21log = "[loop] AHT21: TEMP: " + String(aht21d.temp) + "\tHUM: " + String(aht21d.humidity);

        ENS160Data ens160d = ens160.read();
        String ens160log = "[loop] ENS160: ECO2: " + String(ens160d.eco2)+ "\tTVOC: " + String(ens160d.tvoc);
        
        debugln(aht21Log.c_str());
        debugln(ens160log.c_str());


        JsonDocument json;

        json["timestamp"] = timestamp;
        json["eco2"] = ens160d.eco2;
        json["tvoc"] = ens160d.tvoc;
        json["humidity"] = aht21d.humidity;
        json["temperature"] = aht21d.temp;

        char jsonBuffer[256];
        serializeJson(json, jsonBuffer);

        // Construct the topic with MAC address, removing colons
        String mac = getMqttTopic();
        String topic = String(MQTT_TOPIC) + mac;

        // Publish the JSON string to the constructed topic
        client.publish(topic.c_str(), jsonBuffer);

        debug("[loop] Published message to ");
        debugln(topic.c_str());
    }

    // call loop() regularly to maintain MQTT connection and handle incoming messages
    client.loop();
}
