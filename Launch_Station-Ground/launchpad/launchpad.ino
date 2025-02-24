// importo modulos
#include <esp_now.h>
#include <WiFi.h>



#define ON_LED 22             // LED to tell it has power
#define STATUS_LED 21         // Status arm led
#define RELAY_PIN 23          // Relay


// ESP_NOW
// REPLACE WITH YOUR RECEIVER MAC Address
uint8_t broadcastAddress[] = {0xe0, 0x5a, 0x1b, 0x5f, 0x8c, 0xd8};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
    int ping;
} struct_message;



// Global variables
// Global variables for outgoing and incoming data
struct_message pingMessage;
struct_message receivedPing;

// variable peerInfo para guardar informaciono del receptor(peer)
esp_now_peer_info_t peerInfo;



// Callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Optionally print status:
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// Callback when data is received
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copy the received data into our 'receivedPing' structure
  memcpy(&receivedPing, incomingData, sizeof(receivedPing));
  // Print the received number
  Serial.print("\r\nPing received: ");
  Serial.println(receivedPing.ping);

  // Encender el led
  digitalWrite(STATUS_LED, HIGH);
  delay(1000);
  digitalWrite(STATUS_LED, LOW);

}

// FUNCIONES DE SETUP
// Setup ESP-NOW communication
void setupESPNOW() {
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

void setupOnLed(){
  pinMode(ON_LED, OUTPUT);
  // dejar encendido siempre
  digitalWrite(ON_LED, HIGH);
}

void setupStatusLed(){
  pinMode(STATUS_LED, OUTPUT);
  // dejar como off al inicio
  digitalWrite(STATUS_LED, LOW);
}

void setupRelay(){
  pinMode(RELAY_PIN, OUTPUT);
  // dejar apagado al inicio
  digitalWrite(RELAY_PIN, HIGH);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Hello, ESP32!");

  // setup on_led
  setupOnLed();

  // setup esp_now wifi
  setupESPNOW();

  // setup statusLed
  setupStatusLed();

  // setup relay initially
  setupRelay();
}



// FUNCIONES DE LOOP
// Send a ping message via ESP-NOW
int sendPing() {
  // Set the value to send
  pingMessage.ping = 0;
  
  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&pingMessage, sizeof(pingMessage));
  
  if (result == ESP_OK) {
    return 0;  // Success
  } else {
    return 1;  // Error
  }
}

void loop() {

  // Funcition to send messages
  
  int pingResult = sendPing();

  if (pingResult == 0) {
    Serial.println("Ping sent with success");
  } else {
    Serial.println("Error sending ping");
  }
  
  delay(10000);
}
