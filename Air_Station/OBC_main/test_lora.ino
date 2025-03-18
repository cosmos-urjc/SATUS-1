#include <SPI.h>
#include <RH_RF95.h>
#include <SD.h>
#include <tinycbor.h>

// Define LoRa pins
#define RFM95_CS 10  // Chip Select (CS)
#define RFM95_RST 9  // Reset (RST)
#define RFM95_INT 2  // Interrupt (G0)

// Define microSD card pin
#define SD_CS 4  // Chip Select for microSD card

// Create RFM95 instance
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Simulated sensor data
float temperature = 25.0;
float humidity = 50.0;
float batteryVoltage = 3.7;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  // Manual reset for RFM95
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(100);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize LoRa
  if (!rf95.init()) {
    Serial.println("LoRa initialization failed!");
    while (1);
  }
  Serial.println("LoRa initialized successfully!");

  // Set frequency (e.g., 433 MHz)
  rf95.setFrequency(433.0);

  // Initialize microSD card
  if (!SD.begin(SD_CS)) {
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
  uint8_t cborBuffer[64]; // Adjust buffer size as needed
  size_t cborSize = encodeCBOR(cborBuffer, sizeof(cborBuffer));

  // Send CBOR data via LoRa
  sendData(cborBuffer, cborSize);

  // Save CBOR data to microSD card
  saveToSD(cborBuffer, cborSize);

  delay(5000); // Wait before next cycle
}

// Function to encode telemetry data into CBOR format
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

// Function to send CBOR data via LoRa
void sendData(const uint8_t* data, size_t dataSize) {
  rf95.send(data, dataSize);
  rf95.waitPacketSent();
  Serial.println("CBOR data sent via LoRa");
}

// Function to save CBOR data to microSD card
void saveToSD(const uint8_t* data, size_t dataSize) {
  // Open file on microSD card
  File file = SD.open("telemetry.dat", FILE_WRITE);
  if (file) {
    // Write CBOR data to file
    file.write(data, dataSize);
    file.println(); // Add a newline for readability
    file.close();
    Serial.println("Data saved to microSD card");
  } else {
    Serial.println("Error opening file on microSD card");
  }
}
