// ------------------------------
// Importo módulos y definiciones
// ------------------------------
#include <esp_now.h>
#include <WiFi.h>



// ---------------------
// Definición de Pines
// ---------------------
#define ON_LED 22             // LED indicador de energía - rojo 
#define CONECT_LED 21         // LED de estado (STATE_CONEXION) - amarillo
#define RELAY_PIN 23          // Pin de control para el relay
#define LLAVE_PIN 17          // Pin de la llave
#define BOTON_PIN 18          // Pin del botón
#define ARMED_LED 19          // LED de estado (STATE_ARMED) - verde
#define FIRE_LED 16           // LED de estado (STATE_LANZADO) - azul

// ---------------------
// Variables ESP-NOW
// ---------------------
// Dirección MAC del receptor (reemplaza con la MAC de tu receptor)
uint8_t broadcastAddress[] = {0xa0, 0xa3, 0xb3, 0x29, 0xde, 0x20};
//uint8_t broadcastAddress[] = {0xe0, 0x5a, 0x1b, 0x5f, 0x8c, 0xd8};

// Estructura para enviar datos (debe coincidir con la del receptor)
typedef struct struct_message {
    int arm;
} struct_message;

// Variables globales para envío y recepción
struct_message sendMessage;
struct_message receivedMessage;
// Estructura para la información del peer (dispositivo receptor)
esp_now_peer_info_t peerInfo;

// Variables globales para la sincronización de la entrega del mensaje
volatile bool sendStatusReceived = false;
volatile bool lastSendSuccess   = false;


// -------------------------------------------------------------------------
// Callback: Se ejecuta cuando se envían los datos vía ESP-NOW
// -------------------------------------------------------------------------
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  // Actualiza la bandera según el estado recibido
  lastSendSuccess = (status == ESP_NOW_SEND_SUCCESS);
  // Indica que se ha recibido la respuesta del envío
  sendStatusReceived = true;
  
  // Muestra en el monitor serial el estado de la entrega
  //Serial.print("\r\nEstado del último envío:\t");
  //Serial.println(lastSendSuccess ? "Entrega Exitosa" : "Entrega Fallida");
}



// -------------------------------------------------------------------------
// Callback: Se ejecuta al recibir datos vía ESP-NOW
// -------------------------------------------------------------------------
void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  // Copia los datos recibidos en la estructura "receivedMessage"
  memcpy(&receivedMessage, incomingData, sizeof(receivedMessage));
  
  // Muestra en el monitor serial el valor recibido
  //Serial.print("\r\nPing recibido: ");
  //Serial.println(receivedMessage.arm);
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

void setupConectLed(){
  pinMode(CONECT_LED, OUTPUT);
  // dejar como off al inicio
  digitalWrite(CONECT_LED, LOW); // dejar como off al inicio
}

void setupRelay(){
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, HIGH); // dejar apagado al inicio
}

void setupArmedLed(){
  pinMode(ARMED_LED, OUTPUT);
  digitalWrite(ARMED_LED, LOW); // dejar apagado al inicio
}

void setupBoton(){
  // Configuro el boton como entrada con pull-up interno
  pinMode(BOTON_PIN, INPUT_PULLUP);
}

void setupLlaves(){
  // Configuro la llave como entrada con pull-up interno
  pinMode(LLAVE_PIN, INPUT_PULLUP);
}

void setupFireLed(){
  pinMode(FIRE_LED, OUTPUT);
  digitalWrite(FIRE_LED, LOW); // dejar apagado al inicio
}



// FUNCIONES DE LOOP
// ---------------------
// Función sendArm()
// Envía un "ping" vía ESP-NOW y retorna 1 si se entregó correctamente
// ---------------------
int sendArm() {
  // Asigna el valor del arm a enviar
  sendMessage.arm = 0;
  
  // Reinicia la bandera para indicar que aún no se ha recibido el callback
  sendStatusReceived = false;
  
  // Envía el mensaje vía ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&sendMessage, sizeof(sendMessage));
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






// -------------------------------------------------------------------------
// Máquina de Estados
// -------------------------------------------------------------------------
enum Estado {
  STATE_INICIAL,   // Estado por defecto; espera de conexión (ping)
  STATE_CONEXION,  // Conexión establecida; se enciende CONECT_LED
  STATE_ARMED,     // Estado listo: se monitorea el switch y la llave
  STATE_LANZADO    // Acción final ejecutada (por ejemplo, relay activado)
};

Estado estadoActual = STATE_INICIAL;

// ---------- Funciones de Transición de Estados ----------

void entrarEstadoInicial() {
  estadoActual = STATE_INICIAL;
  Serial.println("Entrando en STATE_INICIAL");
  digitalWrite(CONECT_LED, LOW);
  digitalWrite(ARMED_LED, LOW);
  digitalWrite(FIRE_LED, LOW);
  digitalWrite(RELAY_PIN, HIGH); // Relay apagado
}

void salirEstadoInicial() {
  Serial.println("Saliendo de STATE_INICIAL");
}

void entrarEstadoConexion() {
  estadoActual = STATE_CONEXION;
  Serial.println("Entrando en STATE_CONEXION");
  digitalWrite(CONECT_LED, HIGH);  // Enciende LED de conexión
}

void salirEstadoConexion() {
  Serial.println("Saliendo de STATE_CONEXION");
  digitalWrite(CONECT_LED, LOW);
}

void entrarEstadoArmed() {
  estadoActual = STATE_ARMED;
  Serial.println("Entrando en STATE_ARMED");
  // Aseguramos que en READY el switch y la llave se monitoreen sin activar acción
  digitalWrite(ARMED_LED, HIGH);
}

void salirEstadoArmed() {
  Serial.println("Saliendo de STATE_ARMED");
  digitalWrite(ARMED_LED, LOW);
}

void entrarEstadoLanzado() {
  estadoActual = STATE_LANZADO;
  Serial.println("Entrando en STATE_LANZADO");
  // Activa la acción final (por ejemplo, activa el relay)
  digitalWrite(RELAY_PIN, LOW);  // Activo (relay activo en LOW)
  digitalWrite(FIRE_LED, HIGH);
}

void salirEstadoLanzado() {
  Serial.println("Saliendo de STATE_LANZADO");
  digitalWrite(FIRE_LED, LOW);
}


// -------------------------------------------------------------------------
// Variables para control de temporización
// -------------------------------------------------------------------------
unsigned long previousSendMillis = 0;
const long sendInterval = 1000;  // Intervalo de "ping" en milisegundos


// -------------------------------------------------------------------------
// Función: setup()
// Configura el monitor serial, los periféricos y la comunicación ESP-NOW.
// -------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Serial.println("¡Hola, ESP32!");

  // Inicialización de periféricos (descomentar si se utilizan)
  setupOnLed();
  setupConectLed();
  //setupRelay();
  setupArmedLed();
  setupBoton();
  setupLlaves();
  setupFireLed();

  // Configura la comunicación ESP-NOW
  setupESPNOW();

  // Arrancamos en estado inicial
  entrarEstadoInicial();
}



/************************************************************************************
 * loop()
 ************************************************************************************/
void loop() {
  unsigned long currentMillis = millis();

  // Lectura de llave/boton
  int estadoLlave = digitalRead(LLAVE_PIN);     // HIGH = abierta (off), LOW = cerrada (on)
  int estadoSwitch = digitalRead(BOTON_PIN);    // HIGH = abierta (off), LOW = cerrada (on)

  // 1) Verificar conexión cada 'sendInterval'
  if (currentMillis - previousSendMillis >= sendInterval) {
    previousSendMillis = currentMillis;

    // Resultado de ping: 1 = éxito, 0 = fallo
    int armResult = sendArm();  

    if (armResult == 1) {
      // Se recibió bien el "ping" --> Hay conexión
      if (estadoActual == STATE_INICIAL) {
        // Pasar de "sin conexión" a "con conexión"
        Serial.println("TRANSICIÓN: STATE_INICIAL -> STATE_CONEXION");
        salirEstadoInicial();
        entrarEstadoConexion();
      }
      // Si está en STATE_CONEXION, simplemente me mantengo
    }

    else {
      // Fallo en la entrega => No hay conexión
      if (estadoActual == STATE_CONEXION) {
        // De "conectado" volvemos a "inicial" (sin conexión)
        Serial.println("TRANSICIÓN: STATE_CONEXION -> STATE_INICIAL");
        salirEstadoConexion();
        entrarEstadoInicial();
      }
      else if (estadoActual == STATE_ARMED) {
        // Si está ARMADO y perdemos conexión, desarmar
        Serial.println("TRANSICIÓN: STATE_ARMED -> STATE_INICIAL");
        salirEstadoArmed();
        entrarEstadoInicial();
      }
      else if (estadoActual == STATE_LANZADO) {
        // Si está LANZADO y perdemos conexión, desarmar
        Serial.println("TRANSICIÓN: STATE_LANZADO -> STATE_INICIAL");
        salirEstadoLanzado();
        entrarEstadoInicial();
      }
      // Opcionalmente poner aqui algo si se va la conexion y esta en STATE_LANZADO
    }
  }

  // 2) Máquina de estados (condiciones de llave/botón)
  switch (estadoActual) {
    case STATE_INICIAL:
      // Esperamos a que se establezca la conexión (ping)
      break;

    case STATE_CONEXION:
      // Para armar: La llave debe estar cerrada (LOW) y el botón no pulsado (HIGH).
      if (estadoLlave == LOW && estadoSwitch == HIGH) {
        Serial.println("TRANSICIÓN: STATE_CONEXION -> STATE_ARMED");
        entrarEstadoArmed();
      }
      break;

    case STATE_ARMED:
      // - Si la llave se abre (HIGH), se "desarma" → vuelve a CONEXION
      // - Si la llave sigue cerrada (LOW) y el botón se pulsa (LOW), lanzamos.
      if (estadoLlave == HIGH) {
        Serial.println("TRANSICIÓN: STATE_READY -> STATE_CONEXION");
        salirEstadoArmed();
        entrarEstadoConexion();
      } 
      else if (estadoLlave == LOW && estadoSwitch == LOW) {
        Serial.println("TRANSICIÓN: STATE_ARMED -> STATE_LANZADO");
        //salirEstadoArmed();
        entrarEstadoLanzado();
      }
      break;

    case STATE_LANZADO:
      // - Si la llave se abre (HIGH), se "desarma" → vuelve a CONEXION
      // - Si la llave sigue cerrada (LOW) y el botón se despulsa (HIGH), se vuelve a armado.
      if (estadoLlave == HIGH) {
        Serial.println("TRANSICIÓN: STATE_LANZADO -> STATE_CONEXION");
        salirEstadoLanzado();
        salirEstadoArmed();
        entrarEstadoConexion();
      } 
      if (estadoSwitch == HIGH) {
        Serial.println("TRANSICIÓN: STATE_LANZADO -> STATE_ARMED");
        salirEstadoLanzado();
        entrarEstadoArmed();
      }
      break;
  }
  
  delay(100);  // Pequeño retardo para estabilizar las lecturas
}
