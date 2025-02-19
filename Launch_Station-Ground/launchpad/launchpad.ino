/*
  Testeo de recepcion y transmision de datos wifi con el protocolo esp_now de largo alcanze para ESP32
  Receptor
*/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xa0, 0xa3, 0xb3, 0x29, 0xde, 0x20};

// define the relay pin
#define RELAY_PIN 23

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int a;
} struct_message;

// Global variables for outgoing and incoming data
struct_message myData;
struct_message incomingNumber;

// variable peerInfo para guardar informaciono del receptor(peer)
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  // Copy the received data into our incomingNumber struct
  memcpy(&incomingNumber, incomingData, sizeof(incomingNumber));

  // Print the received number
  Serial.print("\r\nNumber received: ");
  Serial.println(incomingNumber.a);

  // Change the relay status. For example, setting LOW may turn the relay on.
  digitalWrite(RELAY_PIN, LOW);
  Serial.print("RELAY IS ON"); 
  delay(600);
  digitalWrite(RELAY_PIN, HIGH);
}

void setup() {
  // Initialize the relay pin as an OUTPUT
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW); // Set relay OFF initially

  // Init Serial Monitor
  Serial.begin(115200);
 
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register the callback function(cb)
  esp_now_register_send_cb(OnDataSent);
  
  // Register peer (receptor)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;
  
  // Add the peer        
  if (esp_now_add_peer(&peerInfo) != ESP_OK){
    Serial.println("Failed to add peer");
    return;
  }

  // Register for a callback function that will be called when data is received
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}
 
void loop() {
  // Set the value to send
  myData.a = 2;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(20000);
}