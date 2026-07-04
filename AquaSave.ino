/*
 * ============================================================================
 *  AquaSave — Firmware ESP32 (riego automatico + conectividad)
 * ============================================================================
 *  Hardware:
 *    - ESP32 DevKit (WROOM)
 *    - HW-390 sensor capacitivo de humedad de suelo  -> AOUT a GPIO34 (ADC1)
 *    - DHT11 o DHT22 (temperatura / humedad de aire) -> DATA a GPIO16
 *    - Modulo rele 1 canal                           -> IN a GPIO26
 *    - Bomba 5V via rele (COM = +5V fuente externa, NO = rojo bomba)
 *    - Diodo 1N4007 en paralelo con la bomba (flyback)
 *
 *  Notas de cableado del DHT (causa tipica de "t:err / h:err"):
 *    - Si usas el modulo de 3 pines, ya trae resistencia pull-up integrada.
 *    - Si usas el sensor "pelado" de 4 pines, necesita una resistencia de
 *      10k entre VCC y DATA, si no, las lecturas fallan.
 *    - Alimentar el DHT con 3.3V del ESP32 (VCC), GND comun.
 *    - Si tu placa es un ESP32-WROVER, GPIO16 esta ocupado por la PSRAM:
 *      mueve el DATA a otro pin (ej. GPIO4) y cambia PIN_DHT.
 *
 *  Conectividad (arquitectura AquaSave):
 *    ESP32 --MQTT--> Mosquitto --MQTT--> EdgeAPI --HTTP--> Backend AquaSave
 *
 *  Topicos MQTT:
 *    aquasave/devices/<DEVICE_ID>/telemetry     (publica cada 5 s)
 *    aquasave/devices/<DEVICE_ID>/status        (online/offline, retained + LWT)
 *    aquasave/devices/<DEVICE_ID>/commands      (suscrito: open-valve / close-valve)
 *    aquasave/devices/<DEVICE_ID>/commands/ack  (publica confirmacion de comando)
 *
 *  Librerias (Arduino IDE -> Library Manager):
 *    - "DHT sensor library" (Adafruit) + "Adafruit Unified Sensor"
 *    - "PubSubClient" (Nick O'Leary)
 *    - "ArduinoJson" (Benoit Blanchon, v7)
 * ============================================================================
 */

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ─── Configuracion: EDITAR ANTES DE FLASHEAR ────────────────────────────────

// WiFi (red de 2.4 GHz; el ESP32 no soporta 5 GHz)
const char* WIFI_SSID     = "TU_RED_WIFI";
const char* WIFI_PASSWORD = "TU_PASSWORD";

// Broker MQTT (IP de la PC donde corre Mosquitto / el EdgeAPI)
const char* MQTT_HOST = "192.168.1.100";
const uint16_t MQTT_PORT = 1883;

// ID del dispositivo en AquaSave.
// Se obtiene al agregar el dispositivo en la app (detalle del dispositivo ->
// "ID del dispositivo", boton copiar). Debe coincidir exactamente.
const char* DEVICE_ID = "PEGA-AQUI-EL-ID-DEL-DISPOSITIVO";

const char* FIRMWARE_VERSION = "1.1.0";

// ─── Pines ──────────────────────────────────────────────────────────────────

#define PIN_SENSOR_SUELO 34   // HW-390 AOUT (ADC1, compatible con WiFi activo)
#define PIN_DHT          16   // DHT11/DHT22 DATA
#define PIN_RELE         26   // IN del rele (HIGH = bomba ON)

// ─── Calibracion del sensor de suelo ────────────────────────────────────────
// Valor ADC con el sensor al aire (seco total) y sumergido en agua.
// Ajustar si tu sensor da otros valores (verlos en el monitor serie).

const int ADC_SECO_AIRE  = 2600;  // 0 % humedad
const int ADC_MOJADO     = 1100;  // 100 % humedad

// Umbrales de riego automatico (en % de humedad de suelo)
const int SUELO_SECO_PCT   = 35;  // por debajo -> encender bomba
const int SUELO_HUMEDO_PCT = 70;  // por encima -> apagar bomba

// Refuerzo por clima: si hace calor y el aire esta seco, riega aunque el
// suelo este en zona intermedia.
const float TEMP_ALTA          = 30.0;
const float HUMEDAD_AIRE_BAJA  = 40.0;

// Seguridad del modo manual (comando open-valve desde la app)
const unsigned long MANUAL_MAX_MS = 10UL * 60UL * 1000UL;  // 10 min maximo
const int SUELO_SATURADO_PCT      = 90;                    // corte de emergencia

// Caudal nominal de la bomba (L/min). Se reporta al backend para estimar el
// consumo de agua (no hay sensor de flujo en el hardware).
const float CAUDAL_BOMBA_L_MIN = 1.4;

// Intervalos
const unsigned long INTERVALO_LECTURA_MS = 5000;   // ciclo de control + telemetria
const unsigned long INTERVALO_DHT_MS     = 2500;   // el DHT necesita >= 2 s entre lecturas

// ─── DHT: autodeteccion DHT11 / DHT22 ───────────────────────────────────────
// El error clasico "t:err / h:err" o valores absurdos (8 C / 97 %) casi
// siempre es porque el tipo declarado no coincide con el sensor fisico, o
// porque se lee mas rapido de lo que el sensor permite (>= 2 s entre
// lecturas; la libreria devuelve NaN o repite datos viejos si se fuerza).
// Este firmware prueba ambos tipos al arrancar y se queda con el que
// responda con datos validos.

DHT* dht = nullptr;
uint8_t dhtTipoActual = DHT11;      // se resuelve en detectarDHT()
bool dhtDetectado = false;

float ultimaTemperatura = NAN;
float ultimaHumedadAire = NAN;
unsigned long ultimaLecturaDhtMs = 0;
unsigned long ultimoIntentoDeteccionMs = 0;
int fallosDhtSeguidos = 0;

// Si el DHT no respondio al arrancar, se reintenta la deteccion cada 5 min
// (por si estaba mal conectado y lo arreglan sin reiniciar).
const unsigned long REINTENTO_DETECCION_MS = 5UL * 60UL * 1000UL;

// ─── Estado de riego ────────────────────────────────────────────────────────

enum ModoRiego { MODO_AUTO, MODO_MANUAL_ON };

ModoRiego modo = MODO_AUTO;
bool bombaEncendida = false;
unsigned long manualDesdeMs = 0;
unsigned long tiempoAnterior = 0;

// ─── MQTT ───────────────────────────────────────────────────────────────────

WiFiClient wifiClient;
PubSubClient mqtt(wifiClient);

String topicTelemetry;
String topicStatus;
String topicCommands;
String topicCommandsAck;

unsigned long ultimoIntentoMqttMs = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, LOW);  // LOW = bomba apagada

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SENSOR_SUELO, ADC_11db);

  Serial.println();
  Serial.println("====================================");
  Serial.println("      AquaSave ESP32 v" + String(FIRMWARE_VERSION));
  Serial.println("====================================");
  Serial.println("HW-390 AOUT -> GPIO34");
  Serial.println("DHT DATA    -> GPIO16");
  Serial.println("RELE IN     -> GPIO26 (HIGH = bomba ON)");
  Serial.println("====================================");

  detectarDHT();

  const String base = "aquasave/devices/" + String(DEVICE_ID);
  topicTelemetry   = base + "/telemetry";
  topicStatus      = base + "/status";
  topicCommands    = base + "/commands";
  topicCommandsAck = base + "/commands/ack";

  conectarWiFi();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  conectarMqtt();
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOOP
// ═════════════════════════════════════════════════════════════════════════════

void loop() {
  mantenerConexiones();
  mqtt.loop();

  leerDHTSiToca();

  const unsigned long ahora = millis();
  if (ahora - tiempoAnterior >= INTERVALO_LECTURA_MS) {
    tiempoAnterior = ahora;

    const int adcSuelo = analogRead(PIN_SENSOR_SUELO);
    const int humedadSueloPct = adcAporcentaje(adcSuelo);

    actualizarBomba(humedadSueloPct);
    digitalWrite(PIN_RELE, bombaEncendida ? HIGH : LOW);

    imprimirLectura(adcSuelo, humedadSueloPct);
    publicarTelemetria(humedadSueloPct);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  DHT
// ═════════════════════════════════════════════════════════════════════════════

// Prueba una lectura con el tipo dado. Espera lo suficiente entre intentos
// (la libreria de Adafruit ignora lecturas a menos de 2 s de la anterior).
bool probarTipoDHT(uint8_t tipo, const char* nombre) {
  if (dht != nullptr) delete dht;
  dht = new DHT(PIN_DHT, tipo);
  dht->begin();

  for (int intento = 1; intento <= 3; intento++) {
    delay(2500);  // margen sobre los 2 s minimos del sensor
    const float t = dht->readTemperature();
    const float h = dht->readHumidity();
    if (!isnan(t) && !isnan(h) && lecturaCoherente(t, h)) {
      Serial.printf("[DHT] Detectado %s (%.1f C, %.1f %%)\n", nombre, t, h);
      ultimaTemperatura = t;
      ultimaHumedadAire = h;
      ultimaLecturaDhtMs = millis();
      return true;
    }
    Serial.printf("[DHT] %s intento %d sin respuesta valida\n", nombre, intento);
  }
  return false;
}

void detectarDHT() {
  ultimoIntentoDeteccionMs = millis();
  Serial.println("[DHT] Detectando sensor (DHT11 / DHT22)...");

  if (probarTipoDHT(DHT11, "DHT11")) {
    dhtTipoActual = DHT11;
    dhtDetectado = true;
    return;
  }
  if (probarTipoDHT(DHT22, "DHT22")) {
    dhtTipoActual = DHT22;
    dhtDetectado = true;
    return;
  }

  dhtDetectado = false;
  Serial.println("[DHT] ERROR: ningun tipo respondio. Revisa:");
  Serial.println("  1. VCC a 3.3V y GND comun con el ESP32");
  Serial.println("  2. DATA en GPIO16 (en WROVER usa otro pin, ej. GPIO4)");
  Serial.println("  3. Pull-up de 10k entre VCC y DATA (sensor de 4 pines)");
  Serial.println("  4. Cables cortos y firmes (los dupont flojos fallan)");
  Serial.println("  El riego por suelo sigue funcionando sin el DHT.");
}

// Descarta lecturas fisicamente absurdas (ruido electrico tipico cuando el
// tipo de sensor esta mal configurado).
bool lecturaCoherente(float t, float h) {
  return t > -20.0 && t < 60.0 && h >= 1.0 && h <= 100.0;
}

void leerDHTSiToca() {
  if (!dhtDetectado) {
    if (millis() - ultimoIntentoDeteccionMs >= REINTENTO_DETECCION_MS) {
      ultimoIntentoDeteccionMs = millis();
      detectarDHT();
    }
    return;
  }

  const unsigned long ahora = millis();
  if (ahora - ultimaLecturaDhtMs < INTERVALO_DHT_MS) return;
  ultimaLecturaDhtMs = ahora;

  const float t = dht->readTemperature();
  const float h = dht->readHumidity();

  if (!isnan(t) && !isnan(h) && lecturaCoherente(t, h)) {
    ultimaTemperatura = t;
    ultimaHumedadAire = h;
    fallosDhtSeguidos = 0;
    return;
  }

  fallosDhtSeguidos++;
  if (fallosDhtSeguidos >= 10) {
    // Muchas fallas seguidas: puede que se reconectara otro sensor.
    Serial.println("[DHT] 10 fallas seguidas, re-detectando tipo...");
    fallosDhtSeguidos = 0;
    detectarDHT();
  }
}

bool hayLecturaDHT() {
  return !isnan(ultimaTemperatura) && !isnan(ultimaHumedadAire);
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOGICA DE RIEGO
// ═════════════════════════════════════════════════════════════════════════════

int adcAporcentaje(int adc) {
  const long pct = map(adc, ADC_SECO_AIRE, ADC_MOJADO, 0, 100);
  return constrain((int)pct, 0, 100);
}

void actualizarBomba(int humedadSueloPct) {
  // Modo manual: la app forzo el riego (open-valve). Se respeta hasta que
  // llegue close-valve, se cumpla el tiempo maximo o el suelo se sature.
  if (modo == MODO_MANUAL_ON) {
    const bool timeout  = millis() - manualDesdeMs >= MANUAL_MAX_MS;
    const bool saturado = humedadSueloPct >= SUELO_SATURADO_PCT;
    if (timeout || saturado) {
      Serial.println(timeout
        ? "[RIEGO] Manual: tiempo maximo alcanzado, vuelvo a AUTO"
        : "[RIEGO] Manual: suelo saturado, corte de seguridad");
      modo = MODO_AUTO;
      bombaEncendida = false;
    } else {
      bombaEncendida = true;
    }
    return;
  }

  // Modo automatico (misma logica original, ahora en %)
  const bool sueloSeco   = humedadSueloPct <= SUELO_SECO_PCT;
  const bool sueloHumedo = humedadSueloPct >= SUELO_HUMEDO_PCT;

  bool ambienteCalienteSeco = false;
  if (hayLecturaDHT()) {
    ambienteCalienteSeco =
      ultimaTemperatura >= TEMP_ALTA && ultimaHumedadAire <= HUMEDAD_AIRE_BAJA;
  }

  if (sueloSeco) bombaEncendida = true;
  if (!sueloHumedo && ambienteCalienteSeco) bombaEncendida = true;
  if (sueloHumedo) bombaEncendida = false;
}

// ═════════════════════════════════════════════════════════════════════════════
//  CONECTIVIDAD
// ═════════════════════════════════════════════════════════════════════════════

void conectarWiFi() {
  Serial.printf("[WiFi] Conectando a %s", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 20000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Conectado, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Sin conexion (reintento en segundo plano). El riego automatico local sigue activo.");
  }
}

void conectarMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  const String clientId = "aquasave-" + String(DEVICE_ID);
  Serial.printf("[MQTT] Conectando a %s:%u ...\n", MQTT_HOST, MQTT_PORT);

  // Last Will: si el ESP32 se desconecta, el broker avisa "offline".
  const char* willPayload = "{\"status\":\"offline\"}";
  if (mqtt.connect(clientId.c_str(), nullptr, nullptr,
                   topicStatus.c_str(), 1, true, willPayload)) {
    Serial.println("[MQTT] Conectado");
    mqtt.subscribe(topicCommands.c_str(), 1);
    publicarStatusOnline();
  } else {
    Serial.printf("[MQTT] Fallo (rc=%d), reintento en 5 s\n", mqtt.state());
  }
}

void mantenerConexiones() {
  if (WiFi.status() != WL_CONNECTED) {
    // WiFi.begin ya reintenta solo; nada bloqueante aqui.
    return;
  }
  if (!mqtt.connected() && millis() - ultimoIntentoMqttMs >= 5000) {
    ultimoIntentoMqttMs = millis();
    conectarMqtt();
  }
}

void publicarStatusOnline() {
  JsonDocument doc;
  doc["status"] = "online";
  doc["firmwareVersion"] = FIRMWARE_VERSION;
  char buffer[128];
  serializeJson(doc, buffer);
  mqtt.publish(topicStatus.c_str(), buffer, true);  // retained
}

void publicarTelemetria(int humedadSueloPct) {
  if (!mqtt.connected()) return;

  JsonDocument doc;
  doc["soilMoisturePct"] = humedadSueloPct;
  if (hayLecturaDHT()) {
    doc["temperatureC"] = serialized(String(ultimaTemperatura, 1));
    doc["humidityPct"]  = serialized(String(ultimaHumedadAire, 1));
  }
  doc["pumpOn"] = bombaEncendida;
  doc["flowRateLMin"] = CAUDAL_BOMBA_L_MIN;

  char buffer[256];
  serializeJson(doc, buffer);
  mqtt.publish(topicTelemetry.c_str(), buffer, false);
}

// Comandos desde la app (via backend -> EdgeAPI -> MQTT)
void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    Serial.printf("[MQTT] Comando ilegible: %s\n", err.c_str());
    return;
  }

  const char* commandId = doc["commandId"] | "";
  const char* type      = doc["type"] | "";

  Serial.printf("[MQTT] Comando recibido: %s (%s)\n", type, commandId);

  if (strcmp(type, "open-valve") == 0) {
    modo = MODO_MANUAL_ON;
    manualDesdeMs = millis();
    bombaEncendida = true;
    digitalWrite(PIN_RELE, HIGH);
  } else if (strcmp(type, "close-valve") == 0) {
    modo = MODO_AUTO;
    bombaEncendida = false;
    digitalWrite(PIN_RELE, LOW);
  } else {
    Serial.printf("[MQTT] Tipo de comando desconocido: %s\n", type);
    return;
  }

  // Confirmar al EdgeAPI para que marque el comando como ejecutado.
  JsonDocument ack;
  ack["commandId"] = commandId;
  ack["type"] = type;
  char buffer[160];
  serializeJson(ack, buffer);
  mqtt.publish(topicCommandsAck.c_str(), buffer, false);
}

// ═════════════════════════════════════════════════════════════════════════════
//  DEBUG SERIE
// ═════════════════════════════════════════════════════════════════════════════

void imprimirLectura(int adcSuelo, int humedadSueloPct) {
  Serial.println();
  Serial.println("========== NUEVA LECTURA ==========");
  Serial.printf("ADC suelo: %d (%d %%)\n", adcSuelo, humedadSueloPct);

  Serial.print("Estado suelo: ");
  if (humedadSueloPct <= SUELO_SECO_PCT) Serial.println("SECO");
  else if (humedadSueloPct >= SUELO_HUMEDO_PCT) Serial.println("HUMEDO");
  else Serial.println("INTERMEDIO");

  if (hayLecturaDHT()) {
    Serial.printf("Temperatura: %.1f C | Humedad aire: %.1f %%\n",
                  ultimaTemperatura, ultimaHumedadAire);
  } else {
    Serial.println("DHT: SIN LECTURA");
  }

  Serial.printf("Modo: %s\n", modo == MODO_AUTO ? "AUTO" : "MANUAL (app)");
  Serial.printf("Bomba: %s\n", bombaEncendida ? "ENCENDIDA" : "APAGADA");
  Serial.printf("WiFi: %s | MQTT: %s\n",
                WiFi.status() == WL_CONNECTED ? "OK" : "SIN CONEXION",
                mqtt.connected() ? "OK" : "SIN CONEXION");
  Serial.println("===================================");
}
