#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <WiFi.h>
#include <PubSubClient.h>
#include "HT_SSD1306Wire.h"
#include <ArduinoJson.h>

// Configuración de Wi-Fi
const char* ssidPrimaria = "Cybersys_Ext";
const char* passPrimaria = "Sjmahpe122512";

const char* ssidSecundaria = "Cybersys_Tlilkuautli";
const char* passSecundaria = "Sjmahpe122512";

const char* ssidTerciaria = "Cybersys";
const char* passTerciaria = "Sjmahpe122512";

// Configuración del broker MQTT
const char* mqttServer = "192.168.1.57";
const int mqttPort = 1883;
const char* mqttTopic = "lora/received";

// Inicialización de display OLED
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
HardwareSerial gpsSerial(2);
WiFiClient espClient;
PubSubClient client(espClient);

// Configuración LoRa
#define RF_FREQUENCY 915000000   // Hz
#define TX_OUTPUT_POWER 14       // dBm
#define LORA_BANDWIDTH 0         // [0: 125 kHz,
#define LORA_SPREADING_FACTOR 7  // [SF7..SF12]
#define LORA_CODINGRATE 1        // [1: 4/5,]
#define LORA_PREAMBLE_LENGTH 8   // Same for Tx and Rx
#define LORA_SYMBOL_TIMEOUT 0    // Symbols
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define RX_TIMEOUT_VALUE 1000
#define BUFFER_SIZE 100  // Define el tamaño del payload

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

static RadioEvents_t RadioEvents;
int16_t txNumber;
int16_t rssi, rxSize;
bool lora_idle = true;

void connectWiFi() {
  Serial.println("Conectando a Wi-Fi...");
  WiFi.begin(ssidPrimaria, passPrimaria);

  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No se pudo conectar a la red primaria, intentando secundaria...");
    WiFi.begin(ssidSecundaria, passSecundaria);
    startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("No se pudo conectar a la red secundaria, intentando terciaria...");
    WiFi.begin(ssidTerciaria, passTerciaria);
    startAttemptTime = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
      delay(500);
      Serial.print(".");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Conexión Wi-Fi establecida.");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("No se pudo conectar a ninguna red Wi-Fi.");
  }
}

void connectMQTT() {
  while (!client.connected()) {
    Serial.println("Conectando al broker MQTT...");
    if (client.connect("ESP32Client")) {
      Serial.println("Conectado al broker MQTT.");
    } else {
      Serial.print("Fallo al conectar, rc=");
      Serial.print(client.state());
      Serial.println(" Intentando de nuevo en 5 segundos.");
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  iniciarDisplay();
  txNumber = 0;
  rssi = 0;

  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  connectWiFi();
  client.setServer(mqttServer, mqttPort);
  connectMQTT();
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();

  if (lora_idle) {
    lora_idle = false;
    Serial.println("Modo RX activado.");
    Radio.Rx(0);
  }
  Radio.IrqProcess();
}

void iniciarDisplay() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);  // Activa Vext
  display.init();
  display.clear();
  display.display();
  display.setContrast(255);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);  // Configura fuente única
}

void mostrarOLED(const String& mensaje1, const String& mensaje2, const String& mensaje3, const String& mensaje4, const String& mensaje5) {
  display.clear();

  // Dibuja un borde moderno
  display.drawRect(0, 0, 128, 64);   // Rectángulo exterior
  display.drawLine(0, 16, 128, 16);  // Línea separadora superior
  display.drawString(5, 2, "Receptor"); 

  // Muestra la información principal
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 5, mensaje1);  // Línea superior (Latitud)
  display.drawString(0, 15, mensaje2);  
  display.drawString(0, 25, mensaje3);  
  display.drawString(0, 35, mensaje4);  
  display.drawString(0, 45, mensaje5);  


  // Muestra todo en la pantalla
  display.display();
}

void OnRxDone(uint8_t* payload, uint16_t size, int16_t rssi, int8_t snr) {
  JsonDocument doc;
  deserializeJson(doc, payload);
  const char* latitud = doc["latitud"];
  const char* longitud = doc["longitud"];
  const char* fecha = doc["fecha"];
  const char* hora = doc["hora"];
  const char* Rssi = doc["Rssi"];
  rssi = rssi;
  rxSize = size;
  memcpy(rxpacket, payload, size);
  rxpacket[size] = '\0';
  Radio.Sleep();
  Serial.printf("\r\nPaquete recibido: \"%s\" con RSSI %d, longitud %d\r\n", rxpacket, rssi, rxSize);

  if (client.connected()) {
    if (client.publish(mqttTopic, rxpacket)) {
      Serial.println("Mensaje enviado al broker MQTT.");
      String mensaje1 = "Latitud" +String(latitud);
      String mensaje2 = "Longitud" +String(longitud);
      String mensaje3 = "fecha" +String(fecha);
      String mensaje4 = "hora" +String(hora);
      String mensaje5 = "Rssi" +String(Rssi);

      mostrarOLED(mensaje1, mensaje2, mensaje3, mensaje4, mensaje5);
    } else {
      Serial.println("Error al enviar mensaje al broker MQTT.");
    }
  }

  lora_idle = true;
}
