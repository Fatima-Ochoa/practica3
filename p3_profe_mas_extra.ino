#include <WiFi.h>
#include <WebSocketsServer.h>    // arduinoWebSockets (Links2004)
#include <ArduinoJson.h>         // ArduinoJson 7.x (Benoit Blanchon)
#include <ESPmDNS.h>             // mDNS / DNS-SD
#include <Adafruit_NeoPixel.h>
#include <DHT.h>
#include <DFRobotDFPlayerMini.h>

// ==== FreeRTOS ====
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

// ----------------------------------------------------------------------
//                    PROTOTIPOS
// ----------------------------------------------------------------------
void enviarJsonACliente(uint8_t idCliente, const JsonDocument& doc);
void difundirJson(const JsonDocument& doc);
void initServoNativo();
void moverServoNativo(int angulo);
void procesarAudioMexbit(String cmdAudio);
void verificarInactividadAudio();
void reiniciarTimerInactividad();

// ====== CONFIGURACIÓN GENERAL ======
#define SIMULAR_DHT true   // true = datos simulados / false = leer DHT22 real

#ifndef PIN_RGB
#define PIN_RGB 48         // Pin LED RGB NeoPixel 
#endif

Adafruit_NeoPixel rgb(1, PIN_RGB, NEO_GRB + NEO_KHZ800);

// ====== CONFIG WiFi ======
const char* ssid     = "Fafa";
const char* password = "amoajosu16";

// ====== Servidor WebSocket ======
WebSocketsServer servidorWS(81);
const int PIN_LED = 44;

// ====== DFPlayer Mini (GPIO 15 RX, GPIO 16 TX) ======
#define FPS_RX_PIN 15  // ESP32 RX <- DFPlayer TX
#define FPS_TX_PIN 16  // ESP32 TX -> Resistencia 1k -> DFPlayer RX

HardwareSerial myDFPlayerSerial(2);
DFRobotDFPlayerMini myDFPlayer;
bool dfplayer_ok = false;

// ====== Inactividad ======
unsigned long ultimaActividadMs = 0;
int etapaInactividad = 0;

const unsigned long IDLE_1_MIN = 60000;   // 1 min -> 0005.mp3 (Estoy listo)
const unsigned long IDLE_2_MIN = 120000;  // 2 min -> 0006.mp3 (Triste)
const unsigned long IDLE_3_MIN = 180000;  // 3 min -> 0007.mp3 (Roncando)

// ====== Temporizadores y variables ======
volatile bool latidoRecibido = false;
unsigned long ultimoEnvioMs  = 0;
unsigned long CADA_MS        = 250;

// ====== DHT22 ======
#define PIN_DHT   6
#define DHTTYPE   DHT22
DHT dht(PIN_DHT, DHTTYPE);

const unsigned long DHT_MIN_MS_REAL = 2000;
const unsigned long DHT_MIN_MS_SIM  = 500;
float tempC_cache = NAN;
float hum_cache   = NAN;
bool  dht_ok      = false;
unsigned long ultimoDhtMs = 0;

// ====== ADC ======
const int PIN_ADC = 4;

// ====== Sensor Ultrasónico HC-SR04 ======
#define PIN_TRIG 17
#define PIN_ECHO 18

float distanciaCM = -1.0;
bool distanciaOk = false;

unsigned long ultimaMedicionDistancia = 0;
const unsigned long INTERVALO_DISTANCIA_MS = 100;

// ====== Servomotor SG90 ======
#define PIN_SERVO 7
int anguloServo = 90;

// ====== Hostname ======
String nombreHost;

// ======================================================================
//                  MAPEO DE PINES – L298N (11–14)
// ======================================================================
#define IN1 11
#define IN2 12
#define IN3 13
#define IN4 14

static inline void motores_init_pines() {
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

static inline void motorA_stop()     { digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);  }
static inline void motorA_adelante() { digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);  }
static inline void motorA_atras()    { digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH); }

static inline void motorB_stop()     { digitalWrite(IN3, LOW);  digitalWrite(IN4, LOW);  }
static inline void motorB_adelante() { digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);  }
static inline void motorB_atras()    { digitalWrite(IN3, LOW);  digitalWrite(IN4, HIGH); }

static inline void motores_parar_total() { motorA_stop(); motorB_stop(); }

static inline void mover_avanzar()              { motorA_adelante(); motorB_adelante(); }
static inline void mover_retroceder()           { motorA_atras();    motorB_atras();    }
static inline void mover_izquierda()            { motorA_atras();    motorB_adelante(); }
static inline void mover_derecha()              { motorA_adelante(); motorB_atras();    }
static inline void mover_avanzar_izquierda()   { motorA_stop();     motorB_adelante(); }
static inline void mover_avanzar_derecha()      { motorA_adelante(); motorB_stop();     }
static inline void mover_retroceder_izquierda() { motorA_stop();     motorB_atras();    }
static inline void mover_retroceder_derecha()   { motorA_atras();    motorB_stop();     }

// ======================================================================
//         COLA / TASK DE MOTORES (CORE 1)
// ======================================================================
enum class DirMov : uint8_t {
  Parado = 0, Avanzar, Retroceder, Izquierda, Derecha,
  AvanzarIzquierda, AvanzarDerecha, RetrocederIzquierda, RetrocederDerecha, Parar
};

struct CmdMotor {
  DirMov accion;
  uint32_t dur_ms;
  uint32_t t0_ms;
  bool continuo;
};

static QueueHandle_t motorCmdQ = nullptr;
static TaskHandle_t motorTaskHandle = nullptr;

void MotorTask(void* arg) {
  (void)arg;
  CmdMotor cmd{DirMov::Parado, 0, 0, false};
  bool enMovimiento = false;
  uint32_t t_fin = 0;

  for (;;) {
    CmdMotor nuevo;

    if (xQueueReceive(motorCmdQ, &nuevo, (TickType_t)0) == pdTRUE) {
      enMovimiento = false;
      motores_parar_total();

      if (nuevo.accion == DirMov::Parar || nuevo.accion == DirMov::Parado || (!nuevo.continuo && nuevo.dur_ms == 0)) {
        enMovimiento = false;
      } else {
        switch (nuevo.accion) {
          case DirMov::Avanzar: mover_avanzar(); break;
          case DirMov::Retroceder: mover_retroceder(); break;
          case DirMov::Izquierda: mover_izquierda(); break;
          case DirMov::Derecha: mover_derecha(); break;
          case DirMov::AvanzarIzquierda: mover_avanzar_izquierda(); break;
          case DirMov::AvanzarDerecha: mover_avanzar_derecha(); break;
          case DirMov::RetrocederIzquierda: mover_retroceder_izquierda(); break;
          case DirMov::RetrocederDerecha: mover_retroceder_derecha(); break;
          default: motores_parar_total(); break;
        }

        if (nuevo.continuo) {
          enMovimiento = false;
        } else {
          nuevo.t0_ms = millis();
          t_fin = nuevo.t0_ms + nuevo.dur_ms;
          enMovimiento = true;
        }

        JsonDocument ack;
        ack["tipo"] = "mover_inicio";
        difundirJson(ack);
      }
      cmd = nuevo;
    }

    if (enMovimiento) {
      uint32_t ahora = millis();
      if ((int32_t)(ahora - t_fin) >= 0) {
        enMovimiento = false;
        motores_parar_total();

        JsonDocument fin;
        fin["tipo"] = "mover_fin";
        difundirJson(fin);
      }
    }
    vTaskDelay(1);
  }
}

// ======================================================================
//         CONTROL DE SERVO (50Hz / 12-bits)
// ======================================================================
void initServoNativo() {
  analogWriteFrequency(PIN_SERVO, 50);
  analogWriteResolution(PIN_SERVO, 12);
  moverServoNativo(90);
}

void moverServoNativo(int angulo) {
  angulo = constrain(angulo, 0, 180);
  int duty = map(angulo, 0, 180, 102, 491);
  analogWrite(PIN_SERVO, duty);
  anguloServo = angulo;
}

// ======================================================================
//                  SENSOR ULTRASONICO HC-SR04 Y FILTRADO
// ======================================================================
float medirDistanciaCM() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  unsigned long duracion = pulseIn(PIN_ECHO, HIGH, 20000UL);
  if (duracion == 0) return -1.0;

  float distancia = duracion * 0.0343 / 2.0;
  if (distancia < 2.0 || distancia > 400.0) return -1.0;

  return distancia;
}

#define TAM_VENTANA 5
#define UMBRAL_CAMBIO_CM 50.0
#define ALFA_FILTRO 0.3f

float lecturasDistancia[TAM_VENTANA];
int cantidadLecturas = 0;
int indiceLectura = 0;
float distanciaFiltrada = -1.0;

float obtenerMediana() {
  float copia[TAM_VENTANA];
  for (int i = 0; i < cantidadLecturas; i++) copia[i] = lecturasDistancia[i];

  for (int i = 0; i < cantidadLecturas - 1; i++) {
    for (int j = i + 1; j < cantidadLecturas; j++) {
      if (copia[j] < copia[i]) {
        float temporal = copia[i];
        copia[i] = copia[j];
        copia[j] = temporal;
      }
    }
  }

  if (cantidadLecturas % 2 == 1) return copia[cantidadLecturas / 2];
  else return (copia[cantidadLecturas / 2 - 1] + copia[cantidadLecturas / 2]) / 2.0;
}

bool filtrarDistancia(float nuevaLectura) {
  if (nuevaLectura < 2.0 || nuevaLectura > 400.0) return false;

  if (distanciaFiltrada >= 0 && fabs(nuevaLectura - distanciaFiltrada) > UMBRAL_CAMBIO_CM) {
    return false;
  }

  lecturasDistancia[indiceLectura] = nuevaLectura;
  indiceLectura = (indiceLectura + 1) % TAM_VENTANA;
  if (cantidadLecturas < TAM_VENTANA) cantidadLecturas++;

  float mediana = obtenerMediana();
  if (distanciaFiltrada < 0) distanciaFiltrada = mediana;
  else distanciaFiltrada = ALFA_FILTRO * mediana + (1.0 - ALFA_FILTRO) * distanciaFiltrada;

  distanciaCM = distanciaFiltrada;
  return true;
}

// ======================================================================
//                        LÓGICA AUDIO MEXBIT
// ======================================================================
void reiniciarTimerInactividad() {
  ultimaActividadMs = millis();
  if (etapaInactividad > 0) {
    etapaInactividad = 0;
  }
}

void procesarAudioMexbit(String cmdAudio) {
  if (!dfplayer_ok) return;

  reiniciarTimerInactividad();

  myDFPlayer.stop();
  delay(50); 

  if (cmdAudio == "CONNECTING") {
    myDFPlayer.playMp3Folder(2); // 0002.mp3 - Conectando
  } 
  else if (cmdAudio == "BYE") {
    myDFPlayer.playMp3Folder(8); // 0008.mp3 - Bye Bye
  }
}

void verificarInactividadAudio() {
  if (!dfplayer_ok) return;

  unsigned long transcurrido = millis() - ultimaActividadMs;

  if (transcurrido >= IDLE_1_MIN && etapaInactividad < 1) {
    etapaInactividad = 1;
    myDFPlayer.playMp3Folder(5); // 0005.mp3 - Estoy listo
  } 
  else if (transcurrido >= IDLE_2_MIN && etapaInactividad < 2) {
    etapaInactividad = 2;
    myDFPlayer.playMp3Folder(6); // 0006.mp3 - Triste
  } 
  else if (transcurrido >= IDLE_3_MIN && etapaInactividad < 3) {
    etapaInactividad = 3;
    myDFPlayer.playMp3Folder(7); // 0007.mp3 - Roncando
  }
}

// ======================================================================
//                        AUXILIARES GENERALES
// ======================================================================
void setColor(uint8_t r, uint8_t g, uint8_t b) {
  rgb.setPixelColor(0, rgb.Color(r, g, b));
  rgb.show();
}

void actualizarLedProximidad(float distancia, bool valida) {
  if (!valida) setColor(0, 0, 0);
  else if (distancia > 10.0) setColor(0, 255, 0);
  else if (distancia > 5.0)  setColor(255, 180, 0);
  else if (distancia >= 2.0) setColor(255, 0, 0);
  else setColor(0, 0, 0);
}

String macSinDosPuntos() {
  String mac = WiFi.macAddress();
  mac.toLowerCase();
  String out;
  for (char c : mac) if (c != ':') out += c;
  return out;
}

String sufijoMAC6() {
  String m = macSinDosPuntos();
  return (m.length() >= 6) ? m.substring(m.length() - 6) : m;
}

void enviarJsonACliente(uint8_t idCliente, const JsonDocument& doc) {
  String out; serializeJson(doc, out);
  servidorWS.sendTXT(idCliente, out);
}

void difundirJson(const JsonDocument& doc) {
  String out; serializeJson(doc, out);
  servidorWS.broadcastTXT(out);
}

void actualizarLecturaDHT(bool forzar = false) {
  unsigned long ahora = millis();
  unsigned long intervalo = SIMULAR_DHT ? DHT_MIN_MS_SIM : DHT_MIN_MS_REAL;
  if (!forzar && (ahora - ultimoDhtMs < intervalo)) return;

  if (SIMULAR_DHT) {
    tempC_cache = 25.0 + (random(-300, 300) / 100.0);
    hum_cache   = 45.0 + (random(-1000, 1000) / 100.0);
    dht_ok = true;
    ultimoDhtMs = ahora;
  } else {
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    if (isnan(h) || isnan(t)) {
      dht_ok = false;
    } else {
      hum_cache = h;
      tempC_cache = t;
      dht_ok = true;
      ultimoDhtMs = ahora;
    }
  }
}

// ======================================================================
//                      EVENTOS WEBSOCKET
// ======================================================================
void onEventoWS(uint8_t idCliente, WStype_t tipo, uint8_t * payload, size_t longitud) {
  switch (tipo) {
    case WStype_CONNECTED: {
      IPAddress ip = servidorWS.remoteIP(idCliente);
      Serial.printf("[WS] Cliente %u conectado desde %s\n", idCliente, ip.toString().c_str());

      latidoRecibido = true;
      procesarAudioMexbit("CONNECTING");

      JsonDocument msg;
      msg["tipo"]        = "saludo";
      msg["mensaje"]     = "Servidor ESP32-S3 listo";
      msg["puerto"]      = 81;
      msg["nombre_host"] = nombreHost;
      msg["mac"]         = WiFi.macAddress();
      enviarJsonACliente(idCliente, msg);
    } break;

    case WStype_DISCONNECTED: {
      Serial.printf("[WS] Cliente %u desconectado\n", idCliente);

      latidoRecibido = false;
      procesarAudioMexbit("BYE");

      if (motorCmdQ != nullptr) {
        CmdMotor parar{DirMov::Parar, 0, 0, false};
        xQueueSend(motorCmdQ, &parar, 0);
      }
    } break;

    case WStype_TEXT: {
      reiniciarTimerInactividad();

      JsonDocument doc;
      DeserializationError err = deserializeJson(doc, payload, longitud);
      if (err) return;

      const char* tipoMsg = doc["tipo"] | "";

      if (strcmp(tipoMsg, "latido") == 0) {
        latidoRecibido = true;
        JsonDocument ack;
        ack["tipo"]       = "latido_ok";
        ack["tiempo_ms"]  = (uint32_t)millis();
        enviarJsonACliente(idCliente, ack);
      }
      else if (strcmp(tipoMsg, "leer_dht") == 0) {
        actualizarLecturaDHT(true);
        JsonDocument resp;
        resp["tipo"]        = "dht_lectura";
        resp["dht_ok"]      = dht_ok;
        resp["temperatura"] = tempC_cache;
        resp["humedad"]     = hum_cache;
        enviarJsonACliente(idCliente, resp);
      }
      else if (strcmp(tipoMsg, "led") == 0) {
        int estado = doc["estado"] | 0;
        digitalWrite(PIN_LED, estado ? HIGH : LOW);
        JsonDocument resp;
        resp["tipo"] = "led_ack";
        resp["estado"] = estado;
        enviarJsonACliente(idCliente, resp);
      }
      else if (strcmp(tipoMsg, "rgb") == 0) {
        uint8_t r = doc["r"] | 0;
        uint8_t g = doc["g"] | 0;
        uint8_t b = doc["b"] | 0;
        setColor(r, g, b);
        JsonDocument ack;
        ack["tipo"] = "rgb_ack";
        ack["r"] = r; ack["g"] = g; ack["b"] = b;
        enviarJsonACliente(idCliente, ack);
      }
      else if (strcmp(tipoMsg, "servo") == 0) {
        int angulo = doc["angulo"] | 90;
        moverServoNativo(angulo);

        JsonDocument respuesta;
        respuesta["tipo"] = "servo_ack";
        respuesta["angulo"] = anguloServo;
        enviarJsonACliente(idCliente, respuesta);
      }
      else if (strcmp(tipoMsg, "leer_distancia") == 0) {
        float dist = (distanciaOk && distanciaCM >= 0) ? distanciaCM : medirDistanciaCM();
        JsonDocument resp;
        resp["tipo"] = "distancia_lectura";
        resp["d"] = (dist < 0.0) ? 0.0 : dist;
        enviarJsonACliente(idCliente, resp);
      }
      else if (strcmp(tipoMsg, "scan") == 0) {
        int angulo = doc["angulo"] | 90;
        moverServoNativo(angulo);
        delay(500);

        float dist = medirDistanciaCM();
        if (dist > 0) filtrarDistancia(dist);

        JsonDocument resp;
        resp["tipo"] = "scan";
        resp["angulo"] = anguloServo;
        resp["d"] = (distanciaCM >= 0) ? distanciaCM : ((dist < 0.0) ? 0.0 : dist);
        enviarJsonACliente(idCliente, resp);
      }
      else if (strcmp(tipoMsg, "mover") == 0) {
        const char* accionStr = doc["accion"] | "parar";
        uint32_t ms = doc["ms"] | 600;
        bool continuo = doc["continuo"] | false;

        if (ms > 8000) ms = 8000;

        DirMov a = DirMov::Parar;
        if (strcmp(accionStr, "avanzar") == 0) a = DirMov::Avanzar;
        else if (strcmp(accionStr, "retroceder") == 0) a = DirMov::Retroceder;
        else if (strcmp(accionStr, "izquierda") == 0) a = DirMov::Izquierda;
        else if (strcmp(accionStr, "derecha") == 0) a = DirMov::Derecha;
        else if (strcmp(accionStr, "avanzar_izquierda") == 0) a = DirMov::AvanzarIzquierda;
        else if (strcmp(accionStr, "avanzar_derecha") == 0) a = DirMov::AvanzarDerecha;
        else if (strcmp(accionStr, "retroceder_izquierda") == 0) a = DirMov::RetrocederIzquierda;
        else if (strcmp(accionStr, "retroceder_derecha") == 0) a = DirMov::RetrocederDerecha;
        else if (strcmp(accionStr, "parar") == 0) a = DirMov::Parar;

        CmdMotor cmd{a, ms, 0, continuo};
        bool ok = (motorCmdQ && xQueueSend(motorCmdQ, &cmd, 0) == pdTRUE);

        JsonDocument ack2;
        ack2["tipo"] = ok ? "mover_ack" : "mover_err";
        ack2["accion"] = accionStr;
        ack2["ms"] = ms;
        ack2["continuo"] = continuo;
        enviarJsonACliente(idCliente, ack2);
      }
    } break;

    default:
      break;
  }
}

void iniciarMDNSyDNSSD() {
  nombreHost = "esp32-" + sufijoMAC6();
  nombreHost.toLowerCase();

  if (MDNS.begin(nombreHost.c_str())) {
    MDNS.addService("ws", "tcp", 81);
    MDNS.addServiceTxt("ws", "tcp", "proto", "json");
    Serial.println("[mDNS] Iniciado: " + nombreHost + ".local");
  } else {
    Serial.println("[mDNS] Error iniciando mDNS");
  }
}

// ======================================================================
//                               SETUP
// ======================================================================
void setup() {
  Serial.begin(115200);
  delay(200);

  // Inicializar DFPlayer Mini (GPIO 15 RX, GPIO 16 TX)
  myDFPlayerSerial.begin(9600, SERIAL_8N1, FPS_RX_PIN, FPS_TX_PIN);
  delay(500); // Pausa necesaria para estabilización del módulo de audio

  if (myDFPlayer.begin(myDFPlayerSerial)) {
    dfplayer_ok = true;
    delay(200);
    myDFPlayer.volume(25);        // Volumen (0 - 30)
    delay(200);
    myDFPlayer.playMp3Folder(1);  // Reproduce 0001.mp3 al encender el robot
    Serial.println("[DFPLAYER] Inicializado correctamente y reproduciendo 0001.mp3");
  } else {
    Serial.println("[DFPLAYER] Error al conectar con el modulo");
  }

  // Sensor ultrasónico HC-SR04
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  pinMode(PIN_ADC, INPUT);
  analogReadResolution(12);
  randomSeed(analogRead(PIN_ADC));

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  rgb.begin();
  rgb.clear();
  rgb.show();

  if (!SIMULAR_DHT) dht.begin();

  // Conexión WiFi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  Serial.print("Conectando a WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("IP Obtenida: "); Serial.println(WiFi.localIP());

  // Inicializar Servo
  initServoNativo();
  Serial.println("[SERVO] Inicializado en 90 grados");

  iniciarMDNSyDNSSD();

  // Motores
  motores_init_pines();
  motores_parar_total();

  motorCmdQ = xQueueCreate(5, sizeof(CmdMotor));
  if (motorCmdQ) {
    xTaskCreatePinnedToCore(MotorTask, "MotorTask", 4096, nullptr, 2, &motorTaskHandle, 1);
    Serial.println("[MOTOR] Task iniciado en Core 1");
  }

  // Servidor WebSocket
  servidorWS.begin();
  servidorWS.onEvent(onEventoWS);
  Serial.println("Servidor WebSocket en ws://" + nombreHost + ".local:81");

  reiniciarTimerInactividad();
}

// ======================================================================
//                               LOOP
// ======================================================================
void loop() {
  servidorWS.loop();

  unsigned long ahora = millis();
  actualizarLecturaDHT(false);

  // Verificar la inactividad de audio (1, 2 y 3 min)
  verificarInactividadAudio();

  // Medir distancia HC-SR04 cada 100 ms
  if (ahora - ultimaMedicionDistancia >= INTERVALO_DISTANCIA_MS) {
    ultimaMedicionDistancia = ahora;
    float nuevaDistancia = medirDistanciaCM();
    bool lecturaValida = filtrarDistancia(nuevaDistancia);
    distanciaOk = lecturaValida;
    actualizarLedProximidad(distanciaCM, distanciaOk);
  }

  // Enviar telemetría continua a la app en Qt
  if (latidoRecibido && (ahora - ultimoEnvioMs >= CADA_MS)) {
    ultimoEnvioMs = ahora;

    int lecturaADC = analogRead(PIN_ADC);

    JsonDocument msg;
    msg["tipo"]         = "telemetria";
    msg["adc"]          = lecturaADC;
    msg["temperatura"]  = tempC_cache;
    msg["humedad"]      = hum_cache;
    msg["dht_ok"]       = dht_ok;
    msg["tiempo_ms"]    = (uint32_t)ahora;
    msg["distancia_cm"] = distanciaCM;
    msg["distancia_ok"] = distanciaOk;
    msg["angulo_servo"] = anguloServo;

    difundirJson(msg);
  }
}
