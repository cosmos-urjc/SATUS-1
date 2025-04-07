/*************************************************************
 * LAUNCHPAD STATION (Estación de Lanzamiento) - ESP32
 *  
 * Este programa corre en el ESP32 que actúa como estación 
 * de lanzamiento. Se encarga de:
 *   - Mantener conexión ESP-NOW con la estación de tierra (ground).
 *   - Manejar 3 estados:
 *       STATE_INICIAL  ->  sin conexión
 *       STATE_CONEXION ->  conexión establecida
 *       STATE_ARMED    ->  armado (definido por el ground)
 *   - Encender un LED de estado para indicar si está conectado.
 *   - (Opcional) Manejar un relé o LED adicional al armar.
 *   - Salir de ARMED si se pierde la conexión o si el ground manda 
 *     comando=0 (desarmar).
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

// ------------------------------
// Definiciones de pines y periféricos
// ------------------------------
#define LED_POWER       22   // LED indicador de energía (rojo)
#define LED_CONEXION    21   // LED indica conexión establecida (amarillo)
#define PIN_RELE        23   // Pin de control del relé (activo en LOW)


// ------------------------------
// Configuración de ESP-NOW
// ------------------------------
// Dirección MAC del receptor (reemplaza con la MAC de tu receptor)
uint8_t broadcastAddress[] = {0xe0, 0x5a, 0x1b, 0x5f, 0x8c, 0xd8};

// Estructura para enviar datos (debe coincidir con la del receptor)
typedef struct {
  int command;    // 0 = desarmado, 1 = ARMED, 2 = lanzar
} EspNowMessage;

// Datos globales para envío/recepción
EspNowMessage sendData;       
EspNowMessage receivedData;   

// Estructura para la información del peer (dispositivo receptor)
esp_now_peer_info_t peerInfo;

// Variables para resultado de envío
volatile bool sendStatusReceived = false;
volatile bool lastSendSuccess    = false;

// Variable global para almacenar el último comando recibido
volatile int lastCommand = 0; // 0 o 1
volatile int armado = 0;  // 0 o 1

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
  // Guardar en lastCommand lo que mande el ground
  lastCommand = receivedData.command;
}


// -------------------------------------------------------------------------
// Función: setupESPNOW()
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
}


// -------------------------------------------------------------------------
// Configuración de pines (LEDs, relé, etc.)
// -------------------------------------------------------------------------
void setupLeds() {
  pinMode(LED_POWER,  OUTPUT);  digitalWrite(LED_POWER,  HIGH); // invertido: HIGH = off
  pinMode(LED_CONEXION, OUTPUT);  digitalWrite(LED_CONEXION, LOW);  // Led de estado apagado inicial
}

void setupRele() {
  pinMode(PIN_RELE, OUTPUT);
  // Relé inactivo => HIGH
  digitalWrite(PIN_RELE, HIGH);
}


// ---------------------
// Función: sendPing()
// Envía un ping (command=0) y retorna 1 si éxito, 0 si fallo.
// ---------------------
int sendPing() {
  // Asigna el valor del ping a enviar
  sendData.command = 0; // es 0 el launch no comunica si esta en estado de armed o no
  
  // Reinicia la bandera para indicar que aún no se ha recibido el callback
  sendStatusReceived = false;
  lastSendSuccess    = false;
  
  // Envía el mensaje vía ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sendData, sizeof(sendData));
  if (result != ESP_OK) {
    // Error al enviar (el mensaje no pudo encolarse)
    return 0;  
  }
  
  // Espera hasta 1 segundo para que se reciba el callback de envío
  unsigned long startTime = millis();
  while (!sendStatusReceived && (millis() - startTime < 1000)) {
    delay(10);  // Pequeña pausa para evitar bloqueo excesivo
  }
  
  // Si no se recibió el callback en 1 segundo, se considera fallo
  // Retorna 1 únicamente si el envío fue exitoso (según el callback), de lo contrario 0.
  return (sendStatusReceived && lastSendSuccess) ? 1 : 0;
}


// -------------------------------------------------------------------------
// Máquina de Estados
// -------------------------------------------------------------------------
enum Estado {
  STATE_INICIAL,   // Sin conexión
  //STATE_CONEXION,  // Con conexión
  STATE_ARMED,     // Armado
  STATE_LAUNCHED   // Lanzamiento
};

Estado estadoActual = STATE_INICIAL;

// ---------------------
// Funciones de transición
// ---------------------
void entrarEstadoInicial() {
  estadoActual = STATE_INICIAL;
  //Serial.println("Entrando en estado: SIN CONEXIÓN");
  Serial.println("Entrando en STATE_INICIAL.");
  // LED de estado apagado
  //digitalWrite(LED_CONEXION, LOW);
}

void salirEstadoInicial() {
  Serial.println("Saliendo de STATE_INICIAL.");
}

void entrarEstadoConexion() {
  //estadoActual = STATE_CONEXION;
  Serial.println("Entrando en STATE_CONEXION (CONECTADO).");
  // LED de estado encendido
  digitalWrite(LED_CONEXION, HIGH);
  // Al entrar a CONEXION, reseteamos cualquier “armado”
  //armado = 0; 
  //lastCommand = 0; 
}

void salirEstadoConexion() {
  Serial.println("Saliendo de STATE_CONEXION.");
  // borrar variables
  //armado = 0; 
  //lastCommand = 0; 
  // LED de estado apagado
  digitalWrite(LED_CONEXION, LOW);
}

void entrarEstadoArmed() {
  estadoActual = STATE_ARMED;
  Serial.println("Launchpad: STATE_ARMED.");
  // Activo el armado por software
  armado = 1;
  // No encendemos relé todavía, se haría en LAUNCHED
}

void salirEstadoArmed() {
  Serial.println("Launchpad: Saliendo de STATE_ARMED.");
  lastCommand = 0;  // borrar el armado
  // Activo el armado por software
  armado = 0;
}

// Nuevo estado LAUNCHED
void entrarEstadoLaunched() {
  estadoActual = STATE_LAUNCHED;
  Serial.println("Launchpad: STATE_LAUNCHED (lanzamiento).");
  // Aquí sí encendemos relé, si “armado==1”
  if (armado == 1) {
    digitalWrite(PIN_RELE, LOW);  // relé activo
    Serial.println("Relé ACTIVADO - Lanzamiento");
  } else {
    // Si por alguna razón “armado==0”, no lanzamos
    digitalWrite(PIN_RELE, HIGH);
    Serial.println("No se puede lanzar: armado=0");
  }
}

void salirEstadoLaunched() {
  Serial.println("Launchpad: Saliendo de STATE_LAUNCHED.");
  digitalWrite(PIN_RELE, HIGH);  // relé inactivo
  // Podemos o no poner armado=0 aquí, depende si quieres 
  // “desarmar” cuando sales de LAUNCHED
  armado = 0;
}

// -------------------------------------------------------------------------
// Variables para control de temporización (no bloqueante)
// -------------------------------------------------------------------------
// Intervalo de envío de ping
unsigned long previousSendMillis  = 0;
const long sendInterval           = 1000;  


// -------------------------------------------------------------------------
// Función: setup()
// Configura el monitor serial, los periféricos y la comunicación ESP-NOW.
// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("¡Hola, ESP32!");

  // Inicialización de periféricos (descomentar si se utilizan)
  setupLeds();
  setupRele();
  setupEspNow(); 

  // Configura la comunicación ESP-NOW
  entrarEstadoInicial();

  xTaskCreate(pingLoop, "Ping Loop", 1024, NULL, 1, NULL);
}

// -------------------------------------------------------------------------
// Función: pingLoop()
// Envía el "ping" periódicamente.
// -------------------------------------------------------------------------
void pingLoop(void *pvParameters){
  while (true) {
    // pillo el tiempo actual
    unsigned long currentMillis = millis();
    
    // 1) Enviar ping cada “sendInterval” para verificar conexión
    if (currentMillis - previousSendMillis >= sendInterval) {
      previousSendMillis = currentMillis;;
      
      int resultPing = sendPing(); // 1 = éxito, 0 = fallo

      if (resultPing == 1) {
        // Si hay éxito en el envío, hay conexión
        entrarEstadoConexion();
      } else {
        salirEstadoConexion();
      }
    }
  }
  
}


// -------------------------------------------------------------------------
// Función: loop()
// Envía el "ping" periódicamente y muestra el resultado en el monitor serial.
// -------------------------------------------------------------------------
void loop() {

  // 2) Procesar el ultimo comando recibido del ground
  // (0 = "desarmar", 1 = "armar", 2 = "lanzar")
  if (lastCommand == 1) {
    // “armar”
    //if (estadoActual == STATE_CONEXION) {
    if (estadoActual == STATE_INICIAL) {
      entrarEstadoArmed();
    }
    //lastCommand = 0; // consumir el comando
  }

  else if (lastCommand == 0){
    // “desarmar”
    // Si estás en ARMED => vuelve a CONEXION
    if (estadoActual == STATE_ARMED) {
      salirEstadoArmed();
      // vuelvo al modo conexion
      //entrarEstadoConexion();
      entrarEstadoInicial();
    }
    // Si estás en LAUNCHED => Apagar relé y volver a CONEXION
    if (estadoActual == STATE_LAUNCHED) {
      salirEstadoLaunched();
      salirEstadoArmed();
      //entrarEstadoConexion();
      entrarEstadoInicial();
    }
  }

  else if (lastCommand == 2) {
    // “lanzar”
    // Solo tiene sentido si estás en ARMED
    if (estadoActual == STATE_ARMED) {
      entrarEstadoLaunched();
    }
  }
  // 3) (Opcional) cualquier otra lógica local

  delay(100);
}
