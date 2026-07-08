/*
 * ============================================================================
 *  AquaSave — Firmware ESP32 (riego automatico + conectividad)
 * ============================================================================
 *  Conectividad:
 *    ESP32 --MQTT/TLS--> HiveMQ Cloud --> EdgeAPI --HTTP--> Backend AquaSave
 *
 *  WiFi FIJO (v2.4): las credenciales WiFi y el DEVICE_ID se configuran aqui
 *  abajo antes de flashear. El dispositivo se conecta solo al encender; ya no
 *  existe el modo de aprovisionamiento por Access Point.
 *
 *  Librerias: WiFi, WiFiClientSecure, Preferences (core ESP32) +
 *    DHT sensor library (Adafruit) + Adafruit Unified Sensor,
 *    PubSubClient, ArduinoJson v7.
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ─── Configuracion: EDITAR ANTES DE FLASHEAR ────────────────────────────────

// WiFi fijo (red de 2.4 GHz; el ESP32 no soporta 5 GHz)
const char* WIFI_SSID     = "TU_RED_WIFI";
const char* WIFI_PASSWORD = "TU_PASSWORD";

// ID del dispositivo en AquaSave. Se obtiene al crear el huerto en la app:
// Dispositivos -> tarjeta del huerto -> Ver detalles -> "ID del dispositivo"
// -> boton copiar. Debe coincidir exactamente.
const char* DEVICE_ID = "PEGA-AQUI-EL-ID-DEL-HUERTO";

// Broker MQTT en HiveMQ Cloud
const char* MQTT_HOST = "109a1a97e0814454afa8d22b818e2da5.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;

// Credenciales MQTT HiveMQ Cloud
const char* MQTT_USERNAME = "aquasave-edge";
const char* MQTT_PASSWORD_MQTT = "Aquasave123";

const char* FIRMWARE_VERSION = "2.4.0";

// ─── Pines ──────────────────────────────────────────────────────────────────

#define PIN_SENSOR_SUELO 34
#define PIN_DHT          16
#define PIN_RELE         26

// ─── Calibracion del sensor de suelo (AUTOAJUSTABLE) ────────────────────────
// Valores iniciales tipicos del HW-390. El firmware ajusta la ventana con los
// extremos realmente observados y los persiste en NVS: si tu sensor entrega
// valores fuera del rango inicial, la humedad ya no se clava en 0 o 100 %.

const int ADC_SECO_AIRE  = 2600;  // 0 % humedad (valor inicial)
const int ADC_MOJADO     = 1100;  // 100 % humedad (valor inicial)

// Lecturas fuera de este rango = sensor desconectado o ruido electrico: se
// reportan pero NO recalibran la ventana.
const int ADC_VALIDO_MIN = 80;
const int ADC_VALIDO_MAX = 4020;
// Ancho minimo de la ventana observada para confiar en ella.
const int ADC_VENTANA_MIN = 400;

int adcSecoObservado   = ADC_SECO_AIRE;   // maximo ADC visto (mas seco)
int adcMojadoObservado = ADC_MOJADO;      // minimo ADC visto (mas mojado)
int adcSecoGuardado    = 0;               // ultimo valor persistido en NVS
int adcMojadoGuardado  = 0;

// ─── Riego automatico por PULSOS ────────────────────────────────────────────
// En vez de dejar la bomba encendida hasta alcanzar el objetivo (desperdicio
// de agua y bomba "prendida a cada rato"), se riega en pulsos cortos y se
// deja que el agua se absorba antes de volver a evaluar:
//   humedad <= UMBRAL_RIEGO_PCT -> pulso de PULSO_RIEGO_MS -> descanso de
//   ESPERA_TRAS_PULSO_MS -> si sigue seco, otro pulso... hasta estabilizar.

const int UMBRAL_RIEGO_PCT = 40;   // regar por debajo de este % de humedad
const int SUELO_HUMEDO_PCT = 70;   // objetivo: cortar el pulso al llegar aqui

const unsigned long PULSO_RIEGO_MS      = 3UL * 1000UL;         // 3 segundos
// Descanso entre pulsos: 5 min para pruebas/demo. En produccion subir a
// 1 hora (60UL * 60UL * 1000UL) para dar tiempo real de absorcion.
const unsigned long ESPERA_TRAS_PULSO_MS = 5UL * 60UL * 1000UL;  // 5 minutos

// Refuerzo por clima: si hace calor y el aire esta seco, tambien dispara un
// pulso aunque el suelo este en zona intermedia (respetando el descanso).
const float TEMP_ALTA          = 30.0;
const float HUMEDAD_AIRE_BAJA  = 40.0;

const unsigned long MANUAL_MAX_MS = 10UL * 60UL * 1000UL;
const int SUELO_SATURADO_PCT      = 90;

const float CAUDAL_BOMBA_L_MIN = 1.4;

const unsigned long INTERVALO_LECTURA_MS = 5000;
const unsigned long INTERVALO_DHT_MS     = 2500;

// ─── DHT ────────────────────────────────────────────────────────────────────

DHT* dht = nullptr;
uint8_t dhtTipoActual = DHT22;
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

// Pulsos del riego automatico.
unsigned long pulsoInicioMs   = 0;  // inicio del pulso en curso
unsigned long ultimoPulsoFinMs = 0; // fin del ultimo pulso (0 = nunca rego)

// false cuando el ADC lee fuera del rango fisico plausible (sensor
// desconectado o roto). Fail-safe: sin lectura confiable NO hay riego
// automatico (un sensor suelto leeria "0 %" y regaria cada hora por siempre).
bool sensorSueloValido = true;

// Pausa remota (comando pause-device desde la app). Persistida en NVS: un
// dispositivo pausado no riega (ni auto ni manual) hasta recibir
// resume-device, pero sigue reportando telemetria para poder reactivarlo.
bool dispositivoPausado = false;

// NVS: calibracion del suelo + estado de pausa.
Preferences prefs;

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
  Serial.printf ("Device ID   -> %s\n", DEVICE_ID);
  Serial.println("====================================");

  detectarDHT();
  cargarCalibracion();

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

// Corte de seguridad por TIEMPO, evaluado en CADA iteracion del loop (no
// solo en el tick de lectura de 5 s): aunque una operacion se demore, la
// bomba se apaga a tiempo. Toda salida marca el descanso entre pulsos para
// que el modo automatico no vuelva a regar segundos despues.
void seguridadBomba() {
  if (modo == MODO_AUTO && bombaEncendida &&
      millis() - pulsoInicioMs >= PULSO_RIEGO_MS) {
    bombaEncendida   = false;
    ultimoPulsoFinMs = millis();
    digitalWrite(PIN_RELE, LOW);
    Serial.println("[RIEGO] Pulso completo: esperando absorcion del agua");
  }

  if (modo == MODO_MANUAL_ON &&
      millis() - manualDesdeMs >= MANUAL_MAX_MS) {
    modo             = MODO_AUTO;
    bombaEncendida   = false;
    ultimoPulsoFinMs = millis();  // descanso tambien tras el manual
    digitalWrite(PIN_RELE, LOW);
    Serial.println("[RIEGO] Manual: tiempo maximo alcanzado, vuelvo a AUTO");
  }
}

void loop() {
  seguridadBomba();

  mantenerConexiones();
  mqtt.loop();

  leerDHTSiToca();

  const unsigned long ahora = millis();
  if (ahora - tiempoAnterior >= INTERVALO_LECTURA_MS) {
    tiempoAnterior = ahora;

    const int adcSuelo = analogRead(PIN_SENSOR_SUELO);
    const int humedadSueloPct = adcAporcentaje(adcSuelo);
    sensorSueloValido =
      adcSuelo >= ADC_VALIDO_MIN && adcSuelo <= ADC_VALIDO_MAX;

    actualizarBomba(humedadSueloPct);
    digitalWrite(PIN_RELE, bombaEncendida ? HIGH : LOW);

    imprimirLectura(adcSuelo, humedadSueloPct);
    publicarTelemetria(humedadSueloPct, adcSuelo);
  }
}

// ═════════════════════════════════════════════════════════════════════════════
//  DHT (autodeteccion DHT22 / DHT11)
// ═════════════════════════════════════════════════════════════════════════════

bool probarTipoDHT(uint8_t tipo, const char* nombre) {
  if (dht != nullptr) delete dht;
  dht = new DHT(PIN_DHT, tipo);
  dht->begin();

  for (int intento = 1; intento <= 3; intento++) {
    delay(2500);  // el DHT necesita >= 2 s entre lecturas
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
  Serial.println("[DHT] Detectando sensor (DHT22 / DHT11)...");

  if (probarTipoDHT(DHT22, "DHT22")) {
    dhtTipoActual = DHT22;
    dhtDetectado = true;
    return;
  }

  if (probarTipoDHT(DHT11, "DHT11")) {
    dhtTipoActual = DHT11;
    dhtDetectado = true;
    return;
  }

  dhtDetectado = false;
  Serial.println("[DHT] ERROR: ningun tipo respondio. Revisa:");
  Serial.println("  1. VCC a 3.3V y GND comun con el ESP32");
  Serial.println("  2. DATA en GPIO16 (en placas WROVER usar otro pin)");
  Serial.println("  3. Pull-up de 10k entre VCC y DATA (sensor de 4 pines)");
  Serial.println("  El riego por suelo sigue funcionando sin el DHT.");
}

bool lecturaCoherente(float t, float h) {
  return t > -20.0 && t < 60.0 && h >= 10.0 && h <= 100.0;
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
  // La re-deteccion bloquea ~15 s: nunca con la bomba encendida.
  if (fallosDhtSeguidos >= 10 && !bombaEncendida) {
    Serial.println("[DHT] 10 fallas seguidas, re-detectando tipo...");
    fallosDhtSeguidos = 0;
    detectarDHT();
  }
}

bool hayLecturaDHT() {
  return !isnan(ultimaTemperatura) && !isnan(ultimaHumedadAire);
}

// ═════════════════════════════════════════════════════════════════════════════
//  CALIBRACION AUTOAJUSTABLE DEL SENSOR DE SUELO (+ pausa persistida)
// ═════════════════════════════════════════════════════════════════════════════

// Persistir el estado de pausa remota (sobrevive reinicios y apagones).
void guardarPausa(bool pausado) {
  prefs.begin("aquasave", false);
  prefs.putBool("pausado", pausado);
  prefs.end();
}

void cargarCalibracion() {
  prefs.begin("aquasave", true);  // solo lectura
  adcSecoObservado   = prefs.getInt("adcSeco", ADC_SECO_AIRE);
  adcMojadoObservado = prefs.getInt("adcMojado", ADC_MOJADO);
  dispositivoPausado = prefs.getBool("pausado", false);
  prefs.end();
  if (dispositivoPausado) {
    Serial.println("[NVS] Dispositivo en PAUSA (reactivar desde la app)");
  }
  adcSecoGuardado   = adcSecoObservado;
  adcMojadoGuardado = adcMojadoObservado;
  Serial.printf("[CAL] Ventana ADC: seco=%d mojado=%d\n",
                adcSecoObservado, adcMojadoObservado);
}

// Persiste los extremos observados solo cuando cambian de forma apreciable,
// para no desgastar la flash con escrituras cada 5 s.
void guardarCalibracionSiCambio() {
  if (abs(adcSecoObservado - adcSecoGuardado) < 25 &&
      abs(adcMojadoObservado - adcMojadoGuardado) < 25) {
    return;
  }
  prefs.begin("aquasave", false);
  prefs.putInt("adcSeco", adcSecoObservado);
  prefs.putInt("adcMojado", adcMojadoObservado);
  prefs.end();
  adcSecoGuardado   = adcSecoObservado;
  adcMojadoGuardado = adcMojadoObservado;
  Serial.printf("[CAL] Ventana ADC guardada: seco=%d mojado=%d\n",
                adcSecoObservado, adcMojadoObservado);
}

// ═════════════════════════════════════════════════════════════════════════════
//  LOGICA DE RIEGO
// ═════════════════════════════════════════════════════════════════════════════

int adcAporcentaje(int adc) {
  // Auto-ajuste: ampliar la ventana con extremos reales (lecturas validas).
  if (adc >= ADC_VALIDO_MIN && adc <= ADC_VALIDO_MAX) {
    if (adc > adcSecoObservado)   adcSecoObservado = adc;
    if (adc < adcMojadoObservado) adcMojadoObservado = adc;
    guardarCalibracionSiCambio();
  }

  // Si la ventana observada es demasiado angosta (recien calibrando o sensor
  // plano), usar la ventana inicial de referencia.
  int seco   = adcSecoObservado;
  int mojado = adcMojadoObservado;
  if (seco - mojado < ADC_VENTANA_MIN) {
    seco   = ADC_SECO_AIRE;
    mojado = ADC_MOJADO;
  }

  const long pct = map(adc, seco, mojado, 0, 100);
  return constrain((int)pct, 0, 100);
}

void actualizarBomba(int humedadSueloPct) {
  // Dispositivo pausado desde la app: la bomba nunca enciende.
  if (dispositivoPausado) {
    bombaEncendida = false;
    return;
  }

  if (modo == MODO_MANUAL_ON) {
    // (El corte por tiempo maximo lo hace seguridadBomba() en cada loop.)
    const bool saturado =
      sensorSueloValido && humedadSueloPct >= SUELO_SATURADO_PCT;

    if (saturado) {
      Serial.println("[RIEGO] Manual: suelo saturado, corte de seguridad");
      modo = MODO_AUTO;
      bombaEncendida = false;
      ultimoPulsoFinMs = millis();  // descanso tambien tras el manual
    } else {
      bombaEncendida = true;
    }

    return;
  }

  // ── Modo automatico por PULSOS ──────────────────────────────────────
  // Nunca deja la bomba encendida de forma continua: riega PULSO_RIEGO_MS,
  // descansa ESPERA_TRAS_PULSO_MS para que el agua se absorba, y solo si el
  // suelo sigue seco vuelve a regar.

  // Fail-safe: sin lectura confiable del sensor NO se riega en automatico.
  if (!sensorSueloValido) {
    if (bombaEncendida) {
      bombaEncendida   = false;
      ultimoPulsoFinMs = millis();
      Serial.println("[RIEGO] Pulso cortado: lectura del sensor invalida");
    }
    return;
  }

  if (bombaEncendida) {
    // Pulso en curso: cortar al cumplir el tiempo o al llegar al objetivo.
    const bool pulsoCompleto     = millis() - pulsoInicioMs >= PULSO_RIEGO_MS;
    const bool objetivoAlcanzado = humedadSueloPct >= SUELO_HUMEDO_PCT;
    if (pulsoCompleto || objetivoAlcanzado) {
      bombaEncendida   = false;
      ultimoPulsoFinMs = millis();
      Serial.println(objetivoAlcanzado
        ? "[RIEGO] Pulso cortado: humedad objetivo alcanzada"
        : "[RIEGO] Pulso completo: esperando absorcion del agua");
    }
    return;
  }

  const bool sueloSeco   = humedadSueloPct <= UMBRAL_RIEGO_PCT;
  const bool sueloHumedo = humedadSueloPct >= SUELO_HUMEDO_PCT;

  bool ambienteCalienteSeco = false;
  if (hayLecturaDHT()) {
    ambienteCalienteSeco =
      ultimaTemperatura >= TEMP_ALTA && ultimaHumedadAire <= HUMEDAD_AIRE_BAJA;
  }

  const bool necesitaRiego =
    sueloSeco || (!sueloHumedo && ambienteCalienteSeco);
  const bool descansoCumplido =
    ultimoPulsoFinMs == 0 ||
    millis() - ultimoPulsoFinMs >= ESPERA_TRAS_PULSO_MS;

  if (necesitaRiego && descansoCumplido) {
    bombaEncendida = true;
    pulsoInicioMs  = millis();
    Serial.printf("[RIEGO] Pulso automatico iniciado (humedad %d %%)\n",
                  humedadSueloPct);
  }
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
    Serial.println("[WiFi] Sin conexion. Reintentare luego. El riego automatico local sigue activo.");
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
    // La reconexion TLS puede bloquear varios segundos y estirar un pulso;
    // durante un pulso automatico (max 3 s) se pospone. En manual no se
    // pospone: reconectar es la unica via para recibir close-valve.
    if (modo == MODO_AUTO && bombaEncendida) return;
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

void publicarTelemetria(int humedadSueloPct, int adcRaw) {
  if (!mqtt.connected()) return;

  JsonDocument doc;
  doc["soilMoisturePct"] = humedadSueloPct;
  // ADC crudo para diagnostico/calibracion (el edge lo ignora; visible en el
  // web client de HiveMQ).
  doc["adcRaw"] = adcRaw;
  doc["soilSensorOk"] = sensorSueloValido;

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
    if (dispositivoPausado) {
      // Pausado: no regar, pero confirmar el comando para que el backend no
      // lo reintente eternamente.
      Serial.println("[MQTT] open-valve ignorado: dispositivo en pausa");
    } else {
      modo = MODO_MANUAL_ON;
      manualDesdeMs = millis();
      bombaEncendida = true;
      digitalWrite(PIN_RELE, HIGH);
    }
  } else if (strcmp(type, "close-valve") == 0) {
    modo = MODO_AUTO;
    bombaEncendida = false;
    digitalWrite(PIN_RELE, LOW);
    // El usuario detuvo el riego: respetar el descanso entre pulsos para que
    // el modo automatico no vuelva a encender la bomba segundos despues.
    ultimoPulsoFinMs = millis();
  } else if (strcmp(type, "pause-device") == 0) {
    // Pausa remota: apagar bomba y bloquear todo riego hasta resume-device.
    dispositivoPausado = true;
    modo = MODO_AUTO;
    bombaEncendida = false;
    digitalWrite(PIN_RELE, LOW);
    guardarPausa(true);
    Serial.println("[MQTT] Dispositivo PAUSADO remotamente");
  } else if (strcmp(type, "resume-device") == 0) {
    dispositivoPausado = false;
    guardarPausa(false);
    Serial.println("[MQTT] Dispositivo REACTIVADO remotamente");
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
  Serial.printf("Device: %s\n", DEVICE_ID);
  Serial.printf("ADC suelo: %d (%d %%)%s\n", adcSuelo, humedadSueloPct,
                sensorSueloValido ? "" : "  [SENSOR INVALIDO - sin riego auto]");

  Serial.print("Estado suelo: ");
  if (humedadSueloPct <= UMBRAL_RIEGO_PCT) Serial.println("SECO");
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

  Serial.printf("Modo: %s\n",
                dispositivoPausado ? "PAUSADO (app)"
                : modo == MODO_AUTO ? "AUTO"
                : "MANUAL (app)");
  Serial.printf("Bomba: %s\n", bombaEncendida ? "ENCENDIDA" : "APAGADA");

  Serial.printf(
    "WiFi: %s | MQTT: %s\n",
    WiFi.status() == WL_CONNECTED ? "OK" : "SIN CONEXION",
    mqtt.connected() ? "OK" : "SIN CONEXION"
  );

  Serial.println("===================================");
}
