/*
 * ============================================================================
 *  AquaSave — Firmware ESP32 (riego automatico + conectividad)
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ─── Configuracion: EDITAR ANTES DE FLASHEAR ────────────────────────────────

// WiFi 2.4 GHz
const char* WIFI_SSID     = "NOMBRE_WIFI";
const char* WIFI_PASSWORD = "CONTRASENA_WIFI";

// Broker MQTT en HiveMQ Cloud
const char* MQTT_HOST = "109a1a97e0814454afa8d22b818e2da5.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;

// Credenciales MQTT HiveMQ Cloud
const char* MQTT_USERNAME = "aquasave-edge";
const char* MQTT_PASSWORD_MQTT = "Aquasave123";

// ID del dispositivo en AquaSave
const char* DEVICE_ID = "7b2a4306-c686-4610-b6de-606b3b833c61";

const char* FIRMWARE_VERSION = "1.1.0";

// ─── Pines ──────────────────────────────────────────────────────────────────

#define PIN_SENSOR_SUELO 34
#define PIN_DHT          16
#define PIN_RELE         26

// ─── Calibracion del sensor de suelo ────────────────────────────────────────

const int ADC_SECO_AIRE  = 2600;
const int ADC_MOJADO     = 1100;

const int SUELO_SECO_PCT   = 35;
const int SUELO_HUMEDO_PCT = 70;

const float TEMP_ALTA          = 30.0;
const float HUMEDAD_AIRE_BAJA  = 40.0;

const unsigned long MANUAL_MAX_MS = 10UL * 60UL * 1000UL;
const int SUELO_SATURADO_PCT      = 90;

const float CAUDAL_BOMBA_L_MIN = 1.4;

const unsigned long INTERVALO_LECTURA_MS = 5000;
const unsigned long INTERVALO_DHT_MS     = 2500;

// ─── DHT ────────────────────────────────────────────────────────────────────

DHT* dht = nullptr;
uint8_t dhtTipoActual = DHT11;
bool dhtDetectado = false;

float ultimaTemperatura = NAN;
float ultimaHumedadAire = NAN;
unsigned long ultimaLecturaDhtMs = 0;
int fallosDhtSeguidos = 0;

// ─── Estado de riego ────────────────────────────────────────────────────────

enum ModoRiego { MODO_AUTO, MODO_MANUAL_ON };

ModoRiego modo = MODO_AUTO;
bool bombaEncendida = false;
unsigned long manualDesdeMs = 0;
unsigned long tiempoAnterior = 0;

// ─── MQTT ───────────────────────────────────────────────────────────────────

WiFiClientSecure wifiClient;
PubSubClient mqtt(wifiClient);

String topicTelemetry;
String topicStatus;
String topicCommands;
String topicCommandsAck;

unsigned long ultimoIntentoMqttMs = 0;
unsigned long ultimoIntentoWifiMs = 0;

// ═════════════════════════════════════════════════════════════════════════════
//  SETUP
// ═════════════════════════════════════════════════════════════════════════════

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, LOW);

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SENSOR_SUELO, ADC_11db);

  Serial.println();
  Serial.println("====================================");
  Serial.println("      AquaSave ESP32 v" + String(FIRMWARE_VERSION));
  Serial.println("====================================");
  Serial.println("HW-390 AOUT -> GPIO34");
  Serial.println("DHT DATA    -> GPIO16");
  Serial.println("RELE IN     -> GPIO26 (HIGH = bomba ON)");
  Serial.println("MQTT        -> HiveMQ Cloud TLS");
  Serial.println("====================================");

  detectarDHT();

  const String base = "aquasave/devices/" + String(DEVICE_ID);
  topicTelemetry   = base + "/telemetry";
  topicStatus      = base + "/status";
  topicCommands    = base + "/commands";
  topicCommandsAck = base + "/commands/ack";

  conectarWiFi();

  // Para demo: TLS sin validar certificado raiz.
  // Para produccion, lo ideal seria usar el certificado CA.
  wifiClient.setInsecure();

  mqtt.setServer(MQTT_HOST, MQTT_PORT);
  mqtt.setCallback(onMqttMessage);
  mqtt.setBufferSize(512);
  mqtt.setKeepAlive(60);

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

bool probarTipoDHT(uint8_t tipo, const char* nombre) {
  if (dht != nullptr) delete dht;
  dht = new DHT(PIN_DHT, tipo);
  dht->begin();

  for (int intento = 1; intento <= 3; intento++) {
    delay(2500);
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
  Serial.println("[DHT] ERROR: ningun tipo respondio. Revisa cableado.");
}

bool lecturaCoherente(float t, float h) {
  return t > -20.0 && t < 60.0 && h >= 1.0 && h <= 100.0;
}

void leerDHTSiToca() {
  if (!dhtDetectado) return;

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
  Serial.printf("[WiFi] Conectando a %s\n", WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(1000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  const unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 35000) {
    delay(1000);
    Serial.print("[WiFi] Estado: ");
    Serial.println(WiFi.status());
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("[WiFi] Conectado, IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("[WiFi] Sin conexion. Reintentare luego.");
  }
}

void conectarMqtt() {
  if (WiFi.status() != WL_CONNECTED) return;

  const String clientId = "aquasave-" + String(DEVICE_ID);
  Serial.printf("[MQTT] Conectando a HiveMQ %s:%u ...\n", MQTT_HOST, MQTT_PORT);

  const char* willPayload = "{\"status\":\"offline\"}";

  if (mqtt.connect(
        clientId.c_str(),
        MQTT_USERNAME,
        MQTT_PASSWORD_MQTT,
        topicStatus.c_str(),
        1,
        true,
        willPayload
      )) {
    Serial.println("[MQTT] Conectado a HiveMQ Cloud");
    mqtt.subscribe(topicCommands.c_str(), 1);
    publicarStatusOnline();
  } else {
    Serial.printf("[MQTT] Fallo (rc=%d), reintento en 5 s\n", mqtt.state());
  }
}

void mantenerConexiones() {
  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - ultimoIntentoWifiMs >= 10000) {
      ultimoIntentoWifiMs = millis();
      Serial.println("[WiFi] Reintentando conexion...");
      WiFi.disconnect();
      delay(200);
      WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }
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

  mqtt.publish(topicStatus.c_str(), buffer, true);
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

// ═════════════════════════════════════════════════════════════════════════════
//  COMANDOS MQTT
// ═════════════════════════════════════════════════════════════════════════════

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
    Serial.printf(
      "Temperatura: %.1f C | Humedad aire: %.1f %%\n",
      ultimaTemperatura,
      ultimaHumedadAire
    );
  } else {
    Serial.println("DHT: SIN LECTURA");
  }

  Serial.printf("Modo: %s\n", modo == MODO_AUTO ? "AUTO" : "MANUAL (app)");
  Serial.printf("Bomba: %s\n", bombaEncendida ? "ENCENDIDA" : "APAGADA");

  Serial.printf(
    "WiFi: %s | MQTT: %s\n",
    WiFi.status() == WL_CONNECTED ? "OK" : "SIN CONEXION",
    mqtt.connected() ? "OK" : "SIN CONEXION"
  );

  Serial.println("===================================");
}
