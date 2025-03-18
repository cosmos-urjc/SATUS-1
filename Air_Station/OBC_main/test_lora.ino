#include <LoRa.h>
#include <SD.h>
#include <tinycbor.h>

// Define LoRa pins for SX1276
#define LORA_SS 10
#define LORA_RST 9
#define LORA_DIO0 2

// Define microSD card pins
#define SD_SS 4 // Chip Select for microSD card

// Simulated sensor data
float temperature = 25.0;
float humidity = 50.0;
float batteryVoltage = 3.7;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Initialize LoRa
  if (!LoRa.begin(433E6)) { // Use your LoRa frequency
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  Serial.println("LoRa initialized successfully!");

  // Initialize microSD card
  if (!SD.begin(SD_SS)) {
    Serial.println("microSD card initialization failed!");
    while (1);
  }
  Serial.println("microSD card initialized successfully!");
}

void loop() {
  // Simulate sensor readings
  temperature += random(-5, 5) / 10.0;
  humidity += random(-10, 10) / 10.0;
  batteryVoltage -= 0.01;

  // Encode data into CBOR format
  uint8_t cborBuffer[128]; // Adjust buffer size as needed
  size_t cborSize = encodeCBOR(cborBuffer, sizeof(cborBuffer));

  // Transmit data via LoRa
  transmitLoRa(cborBuffer, cborSize);

  // Save data to microSD card
  saveToSD(cborBuffer, cborSize);

  delay(5000); // Wait before next cycle
}

// Function to encode data into CBOR format
size_t encodeCBOR(uint8_t* buffer, size_t bufferSize) {
  CborEncoder encoder;
  CborEncoder mapEncoder;
  cbor_encoder_init(&encoder, buffer, bufferSize, 0);

  // Start encoding a map (key-value pairs)
  cbor_encoder_create_map(&encoder, &mapEncoder, 3); // Map with 3 key-value pairs

  // Add "temperature" key-value pair
  cbor_encode_text_stringz(&mapEncoder, "temperature");
  cbor_encode_float(&mapEncoder, temperature);

  // Add "humidity" key-value pair
  cbor_encode_text_stringz(&mapEncoder, "humidity");
  cbor_encode_float(&mapEncoder, humidity);

  // Add "batteryVoltage" key-value pair
  cbor_encode_text_stringz(&mapEncoder, "batteryVoltage");
  cbor_encode_float(&mapEncoder, batteryVoltage);

  // Close the map
  cbor_encoder_close_container(&encoder, &mapEncoder);

  // Return the size of the encoded CBOR data
  return cbor_encoder_get_buffer_size(&encoder, buffer);
}

// Function to transmit data via LoRa
void transmitLoRa(const uint8_t* data, size_t dataSize) {
  // Ensure LoRa is the only SPI device active
  digitalWrite(SD_SS, HIGH); // Disable microSD card
  digitalWrite(LORA_SS, LOW); // Enable LoRa module

  // Send data via LoRa
  LoRa.beginPacket();
  LoRa.write(data, dataSize);
  LoRa.endPacket();

  Serial.println("CBOR data sent via LoRa");

  digitalWrite(LORA_SS, HIGH); // Disable LoRa module
}

// Function to save data to microSD card
void saveToSD(const uint8_t* data, size_t dataSize) {
  // Ensure microSD card is the only SPI device active
  digitalWrite(LORA_SS, HIGH); // Disable LoRa module
  digitalWrite(SD_SS, LOW); // Enable microSD card

  // Open file on microSD card
  File file = SD.open("telemetry.txt", FILE_WRITE);
  if (file) {
    // Write CBOR data to file
    file.write(data, dataSize);
    file.println(); // Add a newline for readability
    file.close();
    Serial.println("Data saved to microSD card");
  } else {
    Serial.println("Error opening file on microSD card");
  }

  digitalWrite(SD_SS, HIGH); // Disable microSD card
}