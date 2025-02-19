#define RELAY_PIN 23  // Use another GPIO if needed

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
}

void loop() {
  digitalWrite(RELAY_PIN, LOW);  // Relay ON
  delay(2000);
  digitalWrite(RELAY_PIN, HIGH);  // Relay OFF
  delay(2000);
}

