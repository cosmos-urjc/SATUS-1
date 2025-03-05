# Cosmos-Rocket-Team
SATUS-1 On-Board flight computer

Sensors:
- BMP180
- MPU9250 IMU (Giroscope + accelerometer + Magnetometer)
- Adafruit RMF95 LoRA Module

Boards:
- Arduino Nano V3 (For flight Computer)
- Raspberry Pi Zero W (For Ground Station)
- ESP32 CAM
- ESP32 WROOM 2 (For Launch controller)

Hardware:
- Adafruit PowerBoost 500
- Arduino MicroSD Card module
  
![OBC_schemeV3_bb](https://github.com/user-attachments/assets/4eedce25-e2d9-48eb-b214-fdecbe3a7043)

Connections
- BMP180 (I2C):
  * SDA --> Pin A4
  * SCL --> Pin A5
  * VCC --> Pin 3V3
  * GND --> Pin GND
- MPU925 (I2C):
  * SDA --> Pin A4
  * SCL --> Pin A5
  * VCC --> Pin 3V3
  * GN --> Pin GND
- RFM95 LoRA (SPI):
  * Vin --> Pin 5V
  * GND --> Pin GND
  * G0 --> Pin D3
  * SCK --> Pin D13
  * MISO --> Pin D12
  * MOSI --> Pin D11
  * CS --> Pin D4
  * RST --> Pin D2
- MicroSD Module (SPI):
  * GND --> Pin GND
  * VCC --> Pin 5V
  * MISO --> Pin D12
  * MOSI --> Pin D11
  * SCK --> Pin D13
  * CS --> Pin D10
