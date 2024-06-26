### ESP32-C3 | ENS160 + AHT21

This project collects environmental data using the ENS160 and AHT21 sensors and publishes the data to an MQTT broker. It also uses NTP for timestamping the data.

#### Table of Contents
- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation](#installation)
- [Configuration](#configuration)
- [Usage](#usage)
- [License](#license)

#### Hardware Requirements
- ESP32 or compatible microcontroller
- ENS160 sensor
- AHT21 sensor
- WiFi network

#### Software Requirements
- PlatformIO IDE with VSCode

#### Installation
1. **Clone this Repository**:
   - Clone or download this repository.

2. **Install Dependencies**:
   - Open the cloned project in PlatformIO IDE.
   - PlatformIO should automatically install the required libraries defined in `platformio.ini`.

#### Configuration
1. **WiFi Configuration**:
   - Update the `WIFI_SSID` and `WIFI_PASSWORD` constants in `src/main.cpp` with your WiFi credentials.

2. **MQTT Configuration**:
   - Update the `MQTT_HOST`, `MQTT_USER`, and `MQTT_PASWORD` constants in `src/main.cpp` with your MQTT broker details.
   - Ensure the `MQTT_PORT` is correctly set to your MQTT broker's port (default is 1883).

3. **Pin Configuration**:
   - Verify the `SDA_PIN` and `SCL_PIN` constants in `src/main.cpp` match your hardware setup.
   - The `LED_PIN` is set to pin 2 by default.

#### Usage
1. **Build and Upload the Code**:
   - Connect your ESP32 to your computer.
   - Click the `PlatformIO: Upload` button in the PlatformIO IDE to compile and upload the code.

2. **Monitor Serial Output**:
   - Open the Serial Monitor in PlatformIO IDE.
   - You should see debug output indicating the status of the WiFi connection, MQTT connection, and sensor readings.

3. **MQTT Data**:
   - The sensor data is published to the MQTT broker under the topic `sensors/<MAC_ADDRESS>`, where `<MAC_ADDRESS>` is the MAC address of your ESP32 without colons.

#### License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
