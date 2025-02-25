// importo modulos
#include <esp_now.h>
#include <WiFi.h>


// Definición de pines
#define ON_LED 22             // LED indicador de energía
#define STATUS_LED 21         // LED de estado (por ejemplo, para armado)
#define RELAY_PIN 23          // Pin de control para el relay

// defino los estados
enum Estado {
  ESTADO_1,
  ESTADO_2
}


// ESP_NOW
// Dirección MAC del receptor (reemplaza con la MAC de tu receptor)
//uint8_t broadcastAddress[] = {0xa0, 0xa3, 0xb3, 0x29, 0xde, 0x20};
uint8_t broadcastAddress[] = {0xe0, 0x5a, 0x1b, 0x5f, 0x8c, 0xd8};

// Estructura para enviar datos (debe coincidir con la del receptor)
typedef struct struct_message {
    int ping;
} struct_message;



// Variables Globales
// Variables globales para el envío y recepción
struct_message pingMessage;
struct_message receivedPing;

// Estructura para la información del peer (dispositivo receptor)
esp_now_peer_info_t peerInfo;

// -------------------------------------------------------------------------
// Variables globales para la sincronización de la entrega del mensaje
// -------------------------------------------------------------------------
//
// sendStatusReceived: Bandera que indica si ya se ha recibido el callback de
//                     envío (OnDataSent). Se pone en false antes de enviar y
//                     en true cuando el callback es invocado.
//
// lastSendSuccess:    Indica si el envío fue exitoso (true) o fallido (false),
//                     según el valor recibido en el callback.
//
volatile bool sendStatusReceived = false;
volatile bool lastSendSuccess = false;


// -------------------------------------------------------------------------
// Callback: Se ejecuta cuando se envían los datos vía ESP-NOW
// -------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Actualiza la bandera según el estado recibido
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  // Indica que se ha recibido la respuesta del envío
  sendStatusReceived = true;
  
  // Muestra en el monitor serial el estado de la entrega
  Serial.print("\r\nEstado del último envío:\t");
  Serial.println(lastSendSuccess ? "Entrega Exitosa" : "Entrega Fallida");
}



// -------------------------------------------------------------------------
// Callback: Se ejecuta al recibir datos vía ESP-NOW
// -------------------------------------------------------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copia los datos recibidos en la estructura "receivedPing"
  memcpy(&receivedPing, incomingData, sizeof(receivedPing));
  
  // Muestra en el monitor serial el valor recibido
  Serial.print("\r\nPing recibido: ");
  Serial.println(receivedPing.ping);

  // Enciende el LED de energía (STATUS_LED) durante 1 segundo y luego lo apaga
  digitalWrite(STATUS_LED, HIGH);
  delay(600);
  digitalWrite(STATUS_LED, LOW);
}


// FUNCIONES DE SETUP
// -------------------------------------------------------------------------
// Función: setupESPNOW()
// Configura la comunicación ESP-NOW en modo estación (WIFI_STA) y registra
// los callbacks de envío y recepción.
// -------------------------------------------------------------------------
void setupESPNOW() {
  // Establece el modo Wi-Fi en "Station"
  WiFi.mode(WIFI_STA);
  
  // Inicializa ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error al inicializar ESP-NOW");
    return;
  }

  // Registra el callback para el envío de datos
  esp_now_register_send_cb(OnDataSent);

  // Configura la información del peer (receptor)
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;       // Usa el canal por defecto
  peerInfo.encrypt = false;   // Sin cifrado
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Error al agregar el peer");
    return;
  }

  // Registra el callback para la recepción de datos
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}



// -------------------------------------------------------------------------
// Funciones de configuración de periféricos (LED, Relay, etc.)
// Se pueden activar si se desea utilizar estos elementos
// -------------------------------------------------------------------------
void setupOnLed(){
  pinMode(ON_LED, OUTPUT);
  digitalWrite(ON_LED, HIGH); // Inicia apagado
}

void setupStatusLed(){
  pinMode(STATUS_LED, OUTPUT);
  // dejar como off al inicio
  digitalWrite(STATUS_LED, LOW); // dejar como off al inicio
}

void setupRelay(){
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // dejar apagado al inicio
}




// FUNCIONES DE LOOP
// -------------------------------------------------------------------------
// Función: sendPing()
// Envía un mensaje "ping" vía ESP-NOW y espera el callback de envío.
// Retorna 1 si el mensaje se entregó exitosamente, y 0 en cualquier otro caso.
// -------------------------------------------------------------------------
int sendPing() {
  // Asigna el valor del ping a enviar
  pingMessage.ping = 0;
  
  // Reinicia la bandera para indicar que aún no se ha recibido el callback
  sendStatusReceived = false;
  
  // Envía el mensaje vía ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&pingMessage, sizeof(pingMessage));
  if (result != ESP_OK) {
    return 0;  // Error al enviar (el mensaje no pudo encolarse)
  }
  
  // Espera hasta 1 segundo para que se reciba el callback de envío
  unsigned long startTime = millis();
  while (!sendStatusReceived && (millis() - startTime < 1000)) {
    delay(10);  // Pequeña pausa para evitar bloqueo excesivo
  }
  
  // Si no se recibió el callback en 1 segundo, se considera fallo
  if (!sendStatusReceived) {
    return 0;
  }
  
  // Retorna 1 únicamente si el envío fue exitoso (según el callback), de lo contrario 0.
  return lastSendSuccess ? 1 : 0;
}





// Estado actual
Estado estadoActual = ESTADO_1;

// -------------------------------------------------------------------------
// Función: setup()
// Configura el monitor serial, los periféricos y la comunicación ESP-NOW.
// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("¡Hola, ESP32!");

  // Inicialización de periféricos (descomentar si se utilizan)
  setupOnLed();
  setupStatusLed();
  //setupRelay();

  // Configura la comunicación ESP-NOW
  setupESPNOW();
}



// -------------------------------------------------------------------------
// Función: loop()
// Envía el "ping" periódicamente y muestra el resultado en el monitor serial.
// -------------------------------------------------------------------------
void loop() {
  int pingResult = sendPing();

  if (pingResult == 1) {
    Serial.println("Ping entregado exitosamente");
  } else {
    Serial.println("Fallo en la entrega del ping");
  }
  
  delay(8000); // Espera 5 segundos antes de enviar el siguiente ping



  

}
