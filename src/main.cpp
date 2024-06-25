#include <Arduino.h>
#include <ScioSense_ENS16x.h>
#include <Wire.h>

#include "ens16x_i2c_interface.h"

using namespace ScioSense;

#define DEBUG 1

#if DEBUG == 1
#define debug(x) Serial.print(x)
#define debugln(x) Serial.println(x)
#else
#define debug(x)
#define debugln(x)
#endif

#define LED_PIN 2

#define SDA_PIN 8
#define SCL_PIN 9
#define I2C_FREQUENCY 100000  // I2C frequency in Hz (400 kHz)

#define I2C_ADDRESS 0x53
I2cInterface i2c;

// Create sensor objects
ENS160 ens160;

void setup()
{
  debugln("setup..");

  Serial.begin(115200);
  ens160.enableDebugging(Serial);

  // Initialize the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  // Initialize I2C
  Wire.begin(SDA_PIN, SCL_PIN, I2C_FREQUENCY);
  i2c.begin(Wire, I2C_ADDRESS);

  // Initialize ENS160 sensor
  debugln("begin..");
  while (ens160.begin(&i2c) != true)
  {
    Serial.print(".");
    delay(1000);
  }

  debugln("success");
  ens160.startStandardMeasure();
}

void loop()
{

  digitalWrite(LED_PIN, HIGH);
  delay(2000);
  digitalWrite(LED_PIN, LOW);
  delay(2000);

  ens160.wait();

  if (ens160.update() == ENS16x::Result::Ok)
  {
    if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewData))
    {
      Serial.print("AQI UBA:");
      Serial.print((uint8_t)ens160.getAirQualityIndex_UBA());

      Serial.print("\tTVOC:");
      Serial.print(ens160.getTvoc());
      Serial.print("\tECO2:");
      Serial.println(ens160.getEco2());
    }

    if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewGeneralPurposeData))
    {
      Serial.print("RS0:");
      Serial.print(ens160.getRs0());
      Serial.print("\tRS1:");
      Serial.print(ens160.getRs1());
      Serial.print("\tRS2:");
      Serial.print(ens160.getRs2());
      Serial.print("\tRS3:");
      Serial.println(ens160.getRs3());
    }
  }
  delay(2000);
}
