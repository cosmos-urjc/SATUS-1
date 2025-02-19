/*
  Testeo de recepcion y transmision de datos wifi con el protocolo esp_now de largo alcanze para ESP32
  Transmisor
*/

#include <esp_now.h>
#include <WiFi.h>

// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xe0, 0x5a, 0x1b, 0x5f, 0x8c, 0xd8};

// define ground_status_led
#define ground_led 19
// define arm_key_led
//#define arm_led 18
// define launcher_status_led
//#define launcher_led 17

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int a;
} struct_message;

// Global variables for outgoing and incoming data
struct_message myData;
struct_message incomingNumber;

// Variable global
bool ground_status;

// variable peerInfo para guardar informaciono del receptor(peer)
esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&incomingNumber, incomingData, sizeof(incomingNumber));
  Serial.print("Number received: ");
  Serial.println(incomingNumber.a);
}

void setup() {
  // Setup de leds
  pinMode(ground_led, OUTPUT);
  pinWrite(ground_led, LOW); // Estado del led de normal es apagado

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
  myData.a = 1;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));
   
  if (result == ESP_OK) {
    Serial.println("Sent with success");
  }
  else {
    Serial.println("Error sending the data");
  }
  delay(4000);

  if ground_status == True

}