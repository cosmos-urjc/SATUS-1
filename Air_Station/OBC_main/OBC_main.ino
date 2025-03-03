#include "Wire.h"
#include "I2C.h" //Library to configure I2C protocol objects
#include "Adafruit_BMP085.h" // import the Pressure Sensor Library

#include <SPI.h> // library to configure SPI protocol objects
#include <RH_RF95.h> // import the RHM95 LoRA library

Adafruit_BMP085 BMP180; // create sensor object called mySensor

// BMP180 variables
float Temperature; // Variable for holding temp in C
float altitude;

// IMU variables
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

#define G 9.80665

// LoRA variables and definitions
#define RFM95_CS 4 // Slave selector pin declaration
#define RFM95_RST 2 // Reset pin declaratio 
#define RFM95_INT 2 //Interruption pin declaration
#define RF95_FREQ 915.0 // transmission frequency

// Singleton instance of the radio driver
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Define data extracion interval
#define INTERVAL_MS_PRINT 500
unsigned long lastPrintMillis = 0;

/// Extracts the raw data from MPU9250 ///
struct gyroscope_raw {
  int16_t x, y, z;
} gyroscope;

struct accelerometer_raw {
  int16_t x, y, z;
} accelerometer;

struct temperature_raw {
  int16_t value;
} temperature;

/// creates a struct to save normalized MPU9250 data
struct {
  struct {
    float x, y, z;
  } accelerometer, gyroscope;
} normalized;

void setup()
{
  Wire.begin();
  Serial.begin(115200);
  BMP180.begin(); //initialize BMP180

  // Initialize and configure the IMU
  I2CwriteByte(MPU9250_IMU_ADDRESS, 27, GYRO_FULL_SCALE_1000_DPS); // Configure gyroscope range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 28, ACC_FULL_SCALE_2G);        // Configure accelerometer range
  I2CwriteByte(MPU9250_IMU_ADDRESS, 56, 0x01); // Enable interrupt pin for raw data

}

void loop()
{
  unsigned long currentMillis = millis();

  // Reads BMP180 data
  altitude = BMP180.readAltitude(); //Read Altitude
  Temperature = BMP180.readTemperature();

  // check if the IMU is available and read its data
  if (isImuReady()) {
    readRawImu();
    normalize(gyroscope);
    normalize(accelerometer);
  }

  // Print data only every 500ms --> prevent data bombing
  if (currentMillis - lastPrintMillis > INTERVAL_MS_PRINT) {

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


    Serial.print("ALTITUDE:\t");
    Serial.print(altitude);
    Serial.println();
    Serial.print("Temperature:\t");
    Serial.print(Temperature);

    Serial.println();
    lastPrintMillis = currentMillis;
  }
  
  
}
