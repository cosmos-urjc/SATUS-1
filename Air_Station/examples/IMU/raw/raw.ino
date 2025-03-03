// -------------------------------------------------
// Copyright (c) 2022 HiBit <https://www.hibit.dev>
// -------------------------------------------------

#include "Wire.h"
#include "I2C.h"
#include "Adafruit_BMP085.h" // import the Pressure Sensor Library

Adafruit_BMP085 mySensor; // create sensor object called mySensor

float tempC; // Variable for holding temp in C

float tempF; // Variable for holding temp in F

float pressure; //Variable for holding pressure reading

float altitude;


#define MPU9250_IMU_ADDRESS 0x68
#define MPU9250_MAG_ADDRESS 0x0C

#define GYRO_FULL_SCALE_250_DPS  0x00
#define GYRO_FULL_SCALE_500_DPS  0x08
#define GYRO_FULL_SCALE_1000_DPS 0x10
#define GYRO_FULL_SCALE_2000_DPS 0x18

#define ACC_FULL_SCALE_2G  0x00
#define ACC_FULL_SCALE_4G  0x08
#define ACC_FULL_SCALE_8G  0x10
#define ACC_FULL_SCALE_16G 0x18

#define TEMPERATURE_OFFSET 21 // As defined in documentation

#define INTERVAL_MS_PRINT 1000

#define G 9.80665

/// Extracts the raw data from the sensor ///
struct gyroscope_raw {
  int16_t x, y, z;
} gyroscope;

struct accelerometer_raw {
  int16_t x, y, z;
} accelerometer;

struct magnetometer_raw {
  int16_t x, y, z;

  struct {
    int8_t x, y, z;
  } adjustment;
} magnetometer;

struct temperature_raw {
  int16_t value;
} temperature;

/// creates a struct to save normalized data
struct {
  struct {
    float x, y, z;
  } accelerometer, gyroscope, magnetometer;

  float temperature;
} normalized;

unsigned long lastPrintMillis = 0;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  mySensor.begin(); //initialize mySensor

  I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS); // Configure gyroscope range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);        // Configure accelerometer range

  I2CwriteByte(MPU9250_IMU_ADDRESS, 55, 0x02); // Set by pass mode for the magnetometers
  I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x01); // Enable interrupt pin for raw data

  setMagnetometerAdjustmentValues();

  //Start magnetometer
  I2CwriteByte(MPU9250_MAG_ADDRESS, 0x0A, 0x12); // Request continuous magnetometer measurements in 16 bits (mode 1)
}

void loop()
{
  unsigned long currentMillis = millis();
  altitude = mySensor.readAltitude(); //Read Altitude
  pressure = mySensor.readPressure();
  tempC = mySensor.readTemperature();

  if (isImuReady()) {
    readRawImu();

    normalize(gyroscope);
    normalize(accelerometer);
    //normalize(temperature);
  }

  if (isMagnetometerReady()) {
    readRawMagnetometer();

    normalize(magnetometer);
  }

  if (currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) {
    Serial.print("TEMP:\t");
    Serial.print(normalized.temperature, 2);
    //Serial.print("ALTITUDE:\t");
    //Serial.print(altitude,3);
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.print("C");
    Serial.println();

    Serial.print("GYR (");
    Serial.print("\xC2\xB0"); //Print degree symbol
    Serial.print("/s):\t");
    Serial.print(normalized.gyroscope.x, 4);
    Serial.print("\t\t");
    Serial.print(normalized.gyroscope.y, 4);
    Serial.print("\t\t");
    Serial.print(normalized.gyroscope.z, 4);
    Serial.println();

    Serial.print("ACC (m/s^2):\t");
    Serial.print(normalized.accelerometer.x, 4);
    Serial.print("\t\t");
    Serial.print(normalized.accelerometer.y, 4);
    Serial.print("\t\t");
    Serial.print(normalized.accelerometer.z, 4);
    Serial.println();

    Serial.print("MAG (");
    Serial.print("\xce\xbc"); //Print micro symbol
    Serial.print("T):\t");
    Serial.print(normalized.magnetometer.x, 4);
    Serial.print("\t\t");
    Serial.print(normalized.magnetometer.y, 4);
    Serial.print("\t\t");
    Serial.print(normalized.magnetometer.z, 4);
    Serial.println();
    Serial.print("ALTITUDE:\t");
    Serial.print(altitude);
    Serial.println();
    Serial.print("Temperature:\t");
    Serial.print(tempC);

    Serial.println();

    lastPrintMillis = currentMillis;
  }
}
