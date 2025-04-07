/*************************************************************
 *  GROUND STATION (Estación de Tierra) - ESP32
 *  
 *  Este programa corre en el ESP32 que actúa como estación de 
 *  tierra para lanzar un cohete. Se encarga de:
 *    - Mantener conexión ESP-NOW con la estación de lanzamiento 
 *      (otro ESP32) mediante “pings” periódicos.
 *    - Manejar una máquina de estados más completa:
 *        STATE_INICIAL  ->  Sin conexión
 *        STATE_CONEXION ->  Conexión establecida
 *        STATE_ARMED    ->  Sistema armado (llave y botón supervisados)
 *        STATE_LANZADO  ->  Lanzamiento (relé activo)
 *    - Encender y apagar LEDs de indicación (amarillo, verde, azul).
 *    - Controlar un relé para encender el motor del cohete.
 *    - Desarmar automáticamente si se pierde la conexión.
 *
 *  Autor: Javier Ruiz
 *************************************************************/

// ------------------------------
// Inclusión de librerías
// ------------------------------
#include <esp_now.h>
#include <WiFi.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// ---------------------
// Definición de Pines
// ---------------------

// LED
#define LED_POWER       22   // LED indicador de energía (rojo)
#define LED_CONEXION    21   // LED indica conexión establecida (amarillo)
#define LED_ARMED       19   // LED indica armado (verde)
#define LED_LANZADO     16   // LED indica lanzamiento (azul)

// Relé y controles
#define PIN_LLAVE       17   // Pin de llave de seguridad (INPUT_PULLUP)
#define PIN_BOTON       18   // Pin de botón de disparo (INPUT_PULLUP)


// ---------------------
// Variables ESP-NOW
// ---------------------
// Dirección MAC del receptor (reemplaza con la MAC de tu receptor)
uint8_t broadcastAddress[] = {0xa0, 0xa3, 0xb3, 0x29, 0xde, 0x20};
//uint8_t broadcastAddress[] = {0xe0, 0x5a, 0x1b, 0x5f, 0x8c, 0xd8};

// Estructura para enviar datos (debe coincidir con la del receptor)
typedef struct {
  int command;    // 0 = desarmado, 1 = ARMED, 2 = lanzar
} EspNowMessage;

// Variables globales para envío y recepción
EspNowMessage sendData;
EspNowMessage receivedData;

// Estructura para la información del peer (dispositivo receptor)
esp_now_peer_info_t peerInfo;

// Variables globales para la sincronización de la entrega del mensaje
volatile bool sendStatusReceived = false;
volatile bool lastSendSuccess    = false;


// -------------------------------------------------------------------------
// Callback: Se ejecuta cuando se envían los datos vía ESP-NOW
// -------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Actualiza la bandera según el estado recibido
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  // Indica que se ha recibido la respuesta del envío
  sendStatusReceived = true;
}

// -------------------------------------------------------------------------
// Callback: Se ejecuta al recibir datos vía ESP-NOW
// -------------------------------------------------------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copia los datos recibidos en la estructura "receivedMessage"
  memcpy(&receivedData, incomingData, sizeof(receivedData));
}


// -------------------------------------------------------------------------
// Función: setupEspNow()
// Configura la comunicación ESP-NOW en modo estación (WIFI_STA) y registra
// los callbacks de envío y recepción.
// -------------------------------------------------------------------------
void setupEspNow() {
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

  // dejo el valor por default de desactivado
  sendData.command = 0;
}


// -------------------------------------------------------------------------
// Configuración de pines (LEDs, relé, etc.)
// -------------------------------------------------------------------------
void setupLeds() {
  // LED indicador de energía
  pinMode(LED_POWER, OUTPUT);
  digitalWrite(LED_POWER, HIGH); // invertido: HIGH = off

  // LED de conexión
  pinMode(LED_CONEXION, OUTPUT);
  digitalWrite(LED_CONEXION, LOW); // apagado al inicio

  // LED de armado
  pinMode(LED_ARMED, OUTPUT);
  digitalWrite(LED_ARMED, LOW);

  // LED de lanzamiento
  pinMode(LED_LANZADO, OUTPUT);
  digitalWrite(LED_LANZADO, LOW);
}

void setupControles() {
  // Llave y Botón como entradas con pull-up => “cerrado” = LOW, “abierto” = HIGH
  pinMode(PIN_LLAVE, INPUT_PULLUP);
  pinMode(PIN_BOTON, INPUT_PULLUP);
}

// ---------------------
// Función: sendArm()
// Envía un ping (command=0) y retorna 1 si éxito, 0 si fallo.
// ---------------------
int sendArm() {  
  // Reinicia la bandera para indicar que aún no se ha recibido el callback
  sendStatusReceived = false;
  lastSendSuccess    = false;
  
  // Envía el mensaje vía ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sendData, sizeof(sendData));
  if (result != ESP_OK) {
    return 0;  // Error al enviar (el mensaje no pudo encolarse)
  }
  
  // Espera hasta 1 segundo a que se dispare el callback
  unsigned long startTime = millis();
  while (!sendStatusReceived && (millis() - startTime < 1000)) {
    delay(10);
  }
  
  // Retorna 1 si hubo éxito en la entrega, 0 si no
  return (sendStatusReceived && lastSendSuccess) ? 1 : 0;
}


// -------------------------------------------------------------------------
// Máquina de Estados
// -------------------------------------------------------------------------
enum Estado {
  STATE_INICIAL,   // Sin conexión
  //STATE_CONEXION,  // Conexión establecida
  STATE_ARMED,     // Listo para lanzar (llave y botón supervisados)
  STATE_LANZADO    // Proceso de lanzamiento (relé activo)
};

Estado estadoActual = STATE_INICIAL;

// ---------- Funciones de Transición de Estados ----------

void entrarEstadoInicial() {
  estadoActual = STATE_INICIAL;
  Serial.println("Entrando en STATE_INICIAL (sin conexión).");
  // Apagar LEDs de estados y relé
  //digitalWrite(LED_CONEXION, LOW);
  digitalWrite(LED_ARMED,    LOW);
  digitalWrite(LED_LANZADO,  LOW);
}

void salirEstadoInicial() {
  Serial.println("Saliendo de STATE_INICIAL.");
}

void entrarEstadoConexion() {
  //estadoActual = STATE_CONEXION;
  Serial.println("Entrando en STATE_CONEXION (conectado).");
  // Enciende LED de conexión
  digitalWrite(LED_CONEXION, HIGH);
}

void salirEstadoConexion() {
  Serial.println("Saliendo de STATE_CONEXION.");
  digitalWrite(LED_CONEXION, LOW);
}

// Cuando entramos en STATE_ARMED, enviamos command=1 al Launchpad
void entrarEstadoArmed() {
  estadoActual = STATE_ARMED;
  Serial.println("Entr ando en STATE_ARMED (listo).");
  // Enciende LED verde
  digitalWrite(LED_ARMED, HIGH);
  // Avisar al Launchpad que se “arme”
  sendData.command = 1;
  esp_now_send(broadcastAddress, (uint8_t*)&sendData, sizeof(sendData));
}

// Ejemplo: mandar "desamar" (command=0) al launchpad
void salirEstadoArmed() {
  Serial.println("Saliendo de STATE_ARMED.");
  digitalWrite(LED_ARMED, LOW);
  // Avisar al Launchpad que se desarme (command=0)
  sendData.command = 0;
  esp_now_send(broadcastAddress, (uint8_t*)&sendData, sizeof(sendData));
}

// Cuando entramos en STATE_LANZADO, enviamos command=2 al Launchpad
void entrarEstadoLanzado() {
  estadoActual = STATE_LANZADO;
  Serial.println("Entrando en STATE_LANZADO.");
  // Activa el LED azul
  digitalWrite(LED_LANZADO, HIGH);
  // Avisar al Launchpad que “lance” (command=2)
  sendData.command = 2;
  esp_now_send(broadcastAddress, (uint8_t*)&sendData, sizeof(sendData));
}

void salirEstadoLanzado() {
  Serial.println("Saliendo de STATE_LANZADO.");
  digitalWrite(LED_LANZADO, LOW);
}


// -------------------------------------------------------------------------
// Variables para control de pings
// -------------------------------------------------------------------------
unsigned long previousSendMillis =    0;
const long sendInterval =          1000; // Enviar ping cada 1 seg


// -------------------------------------------------------------------------
// Función: setup()
// Configura el monitor serial, los periféricos y la comunicación ESP-NOW.
// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("¡Hola, ESP32!");

   // Configuración de pines
  setupLeds();
  setupControles();

  // Configuración de ESP-NOW
  setupEspNow();

  // Entramos en estado inicial
  entrarEstadoInicial();

  xTaskCreate(pingLoop, "Ping Loop", 1024, NULL, 1, NULL);
}

// -------------------------------------------------------------------------
// Función: pingLoop()
// Envía el "ping" periódicamente.
// -------------------------------------------------------------------------
void pingLoop(void *pvParameters){
  while (true) {
    unsigned long currentMillis = millis();

    // 1) Enviar ping cada "sendInterval" para verificar conexión
    if (currentMillis - previousSendMillis >= sendInterval) {
      previousSendMillis = currentMillis;

      // Resultado de command: 1 = éxito, 0 = fallo
      int armResult = sendArm();  

      if (armResult == 1) {
        // Se recibió bien el "ping" --> Hay conexión
        entrarEstadoConexion();
      } else {
        salirEstadoConexion();

      }
    }
  }
  
}

// volatile bool connected=false;
// -------------------------------------------------------------------------
// loop()
// Envía pings periódicos para verificar conexión y maneja la máquina de estados
// -------------------------------------------------------------------------
void loop() {

  // 2) Lógica de la máquina de estados con llave y botón
  int estadoLlave = digitalRead(PIN_LLAVE);     // HIGH = abierta (off), LOW = cerrada (on)
  int estadoSwitch = digitalRead(PIN_BOTON);    // HIGH = abierta (off), LOW = cerrada (on)
  
  switch (estadoActual) {
  
    case STATE_INICIAL:
      // Esperamos a que se establezca la conexión (ping)
      // Para armar: llave cerrada (LOW), botón suelto (HIGH)
      if ((estadoLlave == LOW) && (estadoSwitch == HIGH)) {
        Serial.println("TRANSICIÓN: STATE_INICIAL -> STATE_ARMED (llave cerrada, botón suelto)");
        salirEstadoInicial();
        entrarEstadoArmed();
      }
      break;

    //case STATE_CONEXION:
      // Para armar: llave cerrada (LOW), botón suelto (HIGH)
      // if ((estadoLlave == LOW) && (estadoSwitch == HIGH)) {
      //   Serial.println("TRANSICIÓN: STATE_CONEXION -> STATE_ARMED (llave cerrada, botón suelto)");
      //   entrarEstadoArmed();
      // }
      //break;

    case STATE_ARMED:
      // Si la llave se abre (HIGH) => volvemos a CONEXION
      if (estadoLlave == HIGH) {
        //Serial.println("TRANSICIÓN: STATE_ARMED -> STATE_CONEXION (llave abierta)");
        Serial.println("TRANSICIÓN: STATE_ARMED -> STATE_INICIAL (llave abierta)");
        salirEstadoArmed();
        //entrarEstadoConexion();
        entrarEstadoInicial()
      } 
      // Si la llave sigue cerrada (LOW) y el botón se pulsa (LOW) => lanzamos
      else if ((estadoLlave == LOW) && (estadoSwitch == LOW)) {
        Serial.println("TRANSICIÓN: STATE_ARMED -> STATE_LANZADO (botón pulsado)");
        entrarEstadoLanzado();
      }
      break;

    case STATE_LANZADO:
      // - Si la llave se abre (HIGH), se "desarma" => vuelve a CONEXION
      if (estadoLlave == HIGH) {
        //Serial.println("TRANSICIÓN: STATE_LANZADO -> STATE_CONEXION (llave abierta)");
        Serial.println("TRANSICIÓN: STATE_LANZADO -> STATE_INICIAL (llave abierta)");
        salirEstadoLanzado();
        salirEstadoArmed();
        //entrarEstadoConexion();
        entrarEstadoInicial();
      }
      // Si el botón se suelta (HIGH) => volver a ARMED 
      else if (estadoSwitch == HIGH) {
        Serial.println("TRANSICIÓN: STATE_LANZADO -> STATE_ARMED (botón suelto)");
        salirEstadoLanzado();
        entrarEstadoArmed();
      }
      break;
      
  }
  
  // 3) Pequeña pausa de estabilidad
  delay(100);
}
