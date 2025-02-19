#include <WiFi.h>
#include <esp_wifi.h>

void readMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  
  if (ret == ESP_OK) {
    Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.begin();

  delay(2000);  // Wait for WiFi hardware to initialize

  Serial.println("Fetching MAC Address:");
  readMacAddress();
}

void loop() {
  // No loop actions needed
}
