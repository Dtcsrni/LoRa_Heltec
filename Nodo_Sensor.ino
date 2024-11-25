#include <TimeLib.h>  // Librería para manejar y ajustar tiempos
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_TinyGPS++.h"
#include "HT_SSD1306Wire.h"
#include <ArduinoJson.h>
#include <Wire.h>

#define RF_FREQUENCY 915000000  // Hz
#define TX_OUTPUT_POWER 5       // dBm

// Configuración LoRa
#define LORA_BANDWIDTH 0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE 1
#define LORA_PREAMBLE_LENGTH 8
#define LORA_SYMBOL_TIMEOUT 0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON false
#define BUFFER_SIZE 100  // Define el tamaño del payload

// Configuración GPS y OLED
#define RXD2 45
#define TXD2 46
#define GPS_BAUD 9600
#define BUFFER_SIZE 100
#define INTERVALO_ACTUALIZACION_GPS 5000
#define PIN_BATERIA 37  // Pin para medir nivel de batería
#define VBAT_READ_CNTRL_PIN 37

// Configuración del pin para activar la lectura de la batería
//#define VBAT_READ_CNTRL_PIN 36   // GPIO para activar la lectura de la batería

// Inicialización de display OLED
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];
RadioEvents_t RadioEvents;
typedef enum { LOWPOWER,
               STATE_RX,
               STATE_TX } Estados;
Estados estado;

// Variables de sistema
int16_t numeroTransmision = 0, Rssi = 0;
double latitud = 0.0, longitud = 0.0;
unsigned long tiempoAnteriorGPS = 0, tiempoActual = 0, tiempoUltimoMensaje = 0;
String fecha, hora;
float nivelBateria = 0.0;

// Configuración de parámetros ADC
#define ADC_MAX_VALUE 4095     // Valor máximo del ADC (para un ADC de 12 bits)
#define REFERENCE_VOLTAGE 3.3  // Voltaje de referencia (ajustar si es necesario)

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

void mostrarOLED(const String &mensaje1, const String &mensaje2, const String &mensaje3, const String &mensaje4, const String &mensaje5) {
  display.clear();

  // Dibuja un borde moderno
  display.drawRect(0, 0, 128, 64);   // Rectángulo exterior
  display.drawLine(0, 16, 128, 16);  // Línea separadora superior
  display.drawString(5, 3, "Sensor");
  // Muestra la información principal
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 15, mensaje1);   // Línea superior (Latitud)
  display.drawString(0, 25, mensaje2);  // Segunda línea (Longitud)
  display.drawLine(0, 30, 128, 40);     // Línea separadora para la hora y batería
  display.drawString(0, 33, mensaje3);  // Línea inferior (Hora)
  display.drawString(0, 45, mensaje4);  // Línea de batería
  display.drawString(0, 55, mensaje5);  // Línea de batería

  // Muestra todo en la pantalla
  display.display();
}

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, RXD2, TXD2);
  iniciarDisplay();

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  Serial.println("Sistema iniciado");
  numeroTransmision = 0;


  RadioEvents.TxTimeout = onTiempoExpirado;

  Radio.Init(&RadioEvents);

  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR, LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // Configurar el pin de control para activar la lectura de la batería
  //digitalWrite(VBAT_READ_CNTRL_PIN, low);  // Habilitar la lectura de la batería
  // pinMode(VBAT_READ_CNTRL_PIN, OUTPUT);


  estado = STATE_TX;
}

void loop() {
  tiempoActual = millis();
  actualizarGPS();
  medirBateria();


  delay(1000);
  numeroTransmision++;
  generarJSON();
  Serial.printf("Enviando paquete: \"%s\"\n", txpacket);
  Radio.Send((uint8_t *)txpacket, strlen(txpacket));

}

void actualizarGPS() {
  if (tiempoActual - tiempoAnteriorGPS >= INTERVALO_ACTUALIZACION_GPS) {
    obtenerPosicionGPS(latitud, longitud, fecha, hora);
    tiempoAnteriorGPS = tiempoActual;

    String mensaje1 = "Latitud: " + String(latitud, 6);
    String mensaje2 = "Longitud: " + String(longitud, 6);
    String mensaje3 = "Hora: " + hora;
    String mensaje4 = "Bateria: " + String(nivelBateria, 1);
    String mensaje5 = "RSSI: " + String(Rssi, 1);

    mostrarOLED(mensaje1, mensaje2, mensaje3, mensaje4, mensaje5);
  }
}

void obtenerPosicionGPS(double &lat, double &lon, String &fechaGPS, String &horaGPS) {
  lat = 0.0;
  lon = 0.0;
  fechaGPS = "N/A";
  horaGPS = "N/A";

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isUpdated()) {
        lat = gps.location.lat();
        lon = gps.location.lng();
      }
      if (gps.date.isUpdated() && gps.time.isUpdated()) {
        fechaGPS = String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year());
        // Ajustar hora UTC a la hora local de Pachuca (UTC-6 estándar o UTC-5 en horario de verano)
        int horaUTC = gps.time.hour();
        int minuto = gps.time.minute();
        int segundo = gps.time.second();

        // Ajuste para UTC -6
        horaUTC = (horaUTC - 6 + 24) % 24;

        char horaLocal[9];  // Buffer para almacenar la hora en formato HH:MM:SS
        sprintf(horaLocal, "%02d:%02d:%02d", horaUTC, minuto, segundo);
        horaGPS = String(horaLocal);
      }
    }
  }
}

void medirBateria() {
  int lectura = analogRead(PIN_BATERIA);
  nivelBateria = 0.0041 * lectura;
}

void generarJSON() {
  StaticJsonDocument<BUFFER_SIZE> jsonDoc;
  jsonDoc["latitud"] = latitud;
  jsonDoc["longitud"] = longitud;
  jsonDoc["fecha"] = fecha;
  jsonDoc["hora"] = hora;
  jsonDoc["Rssi"] = Rssi;
  serializeJson(jsonDoc, txpacket, BUFFER_SIZE);
}


void onTiempoExpirado() {
  Radio.Sleep();
  Serial.println("Tiempo de transmisión expirado");
  estado = STATE_TX;
}


