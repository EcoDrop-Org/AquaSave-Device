/*
 * ============================================================================
 *  AquaSave — Firmware ESP32 (riego automatico + conectividad + provisioning)
 * ============================================================================
 *  Conectividad:
 *    ESP32 --MQTT/TLS--> HiveMQ Cloud --> EdgeAPI --HTTP--> Backend AquaSave
 *
 *  Aprovisionamiento WiFi (v2.0):
 *    Al arrancar sin credenciales, el ESP32 crea la red "AquaSave-XXXX" y
 *    sirve un portal HTTP en 192.168.4.1 (GET /info, /scan; POST /connect,
 *    /reset). La app entrega WiFi + deviceId; se guardan en NVS. Mantener
 *    BOOT (GPIO0) 3 s borra las credenciales y vuelve a modo AP.
 *
 *  Librerias: WiFi, WiFiClientSecure, WebServer, Preferences (core ESP32) +
 *    DHT sensor library, PubSubClient, ArduinoJson v7.
 * ============================================================================
 */

#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <DHT.h>

// ─── Configuracion: EDITAR ANTES DE FLASHEAR ────────────────────────────────

// Broker MQTT en HiveMQ Cloud
const char* MQTT_HOST = "109a1a97e0814454afa8d22b818e2da5.s1.eu.hivemq.cloud";
const uint16_t MQTT_PORT = 8883;

// Credenciales MQTT HiveMQ Cloud
const char* MQTT_USERNAME = "aquasave-edge";
const char* MQTT_PASSWORD_MQTT = "Aquasave123";

const char* FIRMWARE_VERSION = "2.3.1";

// ─── Aprovisionamiento WiFi (Access Point + portal HTTP) ────────────────────
// El WiFi y el DEVICE_ID YA NO se hardcodean: los entrega la app AquaSave.
// Si el ESP32 arranca sin credenciales guardadas, crea su propia red
// "AquaSave-XXXX" y levanta un servidor HTTP en 192.168.4.1 con:
//   GET  /info    -> identidad del dispositivo
//   GET  /scan    -> redes WiFi reales al alcance
//   POST /connect -> { ssid, password, deviceId } (prueba, guarda y reinicia)
//   POST /reset   -> borra credenciales
// Las credenciales se guardan en NVS y sobreviven al corte de energia.
// Manten pulsado BOOT (GPIO0) 3 s para borrar y re-aprovisionar.

const char* AP_PREFIX   = "AquaSave-";  // se le anexa el sufijo de la MAC
const char* AP_PASSWORD = "aquasave123"; // clave del AP (>= 8 chars)

// Estas variables reemplazan a las antiguas constantes hardcodeadas.
// Se rellenan desde NVS (o desde /connect al aprovisionar).
String WIFI_SSID;
String WIFI_PASSWORD;
String DEVICE_ID;

Preferences prefs;
WebServer apServer(80);
bool provisioningMode = false;
unsigned long bootPressedSince = 0;

// ─── Pines ──────────────────────────────────────────────────────────────────

#define PIN_SENSOR_SUELO 34
#define PIN_DHT          16
#define PIN_RELE         26
#define PIN_BOOT         0    // boton BOOT: mantener 3 s = re-aprovisionar

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

const unsigned long PULSO_RIEGO_MS      = 60UL * 1000UL;        // 1 minuto
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
  pinMode(PIN_BOOT, INPUT_PULLUP);

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
  Serial.println("BOOT (GPIO0): mantener 3 s = re-aprovisionar");
  Serial.println("====================================");

  detectarDHT();

  // Cargar credenciales guardadas (WiFi + deviceId) y calibracion del suelo.
  cargarCredenciales();
  cargarCalibracion();

  if (WIFI_SSID.length() == 0 || DEVICE_ID.length() == 0) {
    // Sin configurar -> modo aprovisionamiento (Access Point + portal HTTP).
    iniciarModoAP();
    return;  // en modo AP el loop() solo atiende el servidor HTTP
  }

  const String base = "aquasave/devices/" + DEVICE_ID;
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
  // En modo aprovisionamiento el ESP32 solo sirve el portal HTTP del AP.
  if (provisioningMode) {
    apServer.handleClient();
    return;
  }

  vigilarBotonReset();
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
//  APROVISIONAMIENTO WiFi (Access Point + servidor HTTP)
// ═════════════════════════════════════════════════════════════════════════════

// Sufijo unico derivado de la MAC (ultimos 2 bytes en HEX): "3F7A".
String sufijoMac() {
  uint8_t mac[6];
  WiFi.macAddress(mac);
  char buf[5];
  snprintf(buf, sizeof(buf), "%02X%02X", mac[4], mac[5]);
  return String(buf);
}

void cargarCredenciales() {
  prefs.begin("aquasave", true);  // solo lectura
  WIFI_SSID     = prefs.getString("ssid", "");
  WIFI_PASSWORD = prefs.getString("pass", "");
  DEVICE_ID     = prefs.getString("deviceId", "");
  prefs.end();

  if (WIFI_SSID.length() > 0) {
    Serial.printf("[NVS] Credenciales: SSID='%s', deviceId='%s'\n",
                  WIFI_SSID.c_str(), DEVICE_ID.c_str());
  } else {
    Serial.println("[NVS] Sin credenciales guardadas.");
  }
}

void guardarCredenciales(const String& ssid, const String& pass, const String& id) {
  prefs.begin("aquasave", false);  // lectura/escritura
  prefs.putString("ssid", ssid);
  prefs.putString("pass", pass);
  prefs.putString("deviceId", id);
  prefs.end();
  Serial.println("[NVS] Credenciales guardadas.");
}

void borrarCredenciales() {
  prefs.begin("aquasave", false);
  prefs.clear();
  prefs.end();
  Serial.println("[NVS] Credenciales borradas. Reiniciando a modo AP...");
  delay(500);
  ESP.restart();
}

void enviarCors() {
  apServer.sendHeader("Access-Control-Allow-Origin", "*");
  apServer.sendHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
  apServer.sendHeader("Access-Control-Allow-Headers", "Content-Type");
}

// Responde el preflight OPTIONS (necesario para la app WEB en navegador).
void handleCors() {
  if (apServer.method() == HTTP_OPTIONS) {
    enviarCors();
    apServer.send(204);
  } else {
    enviarCors();
    apServer.send(404, "application/json", "{\"error\":\"not found\"}");
  }
}

// GET /info -> identidad del dispositivo.
void handleInfo() {
  enviarCors();
  JsonDocument doc;
  doc["ap"] = String(AP_PREFIX) + sufijoMac();
  doc["mac"] = WiFi.macAddress();
  doc["firmware"] = FIRMWARE_VERSION;
  doc["provisioned"] = (WIFI_SSID.length() > 0);
  doc["deviceId"] = DEVICE_ID;  // vacio si aun no se aprovisiono
  String out;
  serializeJson(doc, out);
  apServer.send(200, "application/json", out);
}

// GET /scan -> lista de redes WiFi reales al alcance del ESP32.
void handleScan() {
  enviarCors();
  Serial.println("[AP] Escaneando redes WiFi...");
  const int n = WiFi.scanNetworks();

  JsonDocument doc;
  JsonArray arr = doc.to<JsonArray>();
  for (int i = 0; i < n; i++) {
    JsonObject o = arr.add<JsonObject>();
    o["ssid"] = WiFi.SSID(i);
    o["rssi"] = WiFi.RSSI(i);            // dBm (mas cerca de 0 = mejor senal)
    o["secure"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
  }
  WiFi.scanDelete();

  String out;
  serializeJson(doc, out);
  Serial.printf("[AP] %d redes encontradas.\n", n);
  apServer.send(200, "application/json", out);
}

// POST /connect { ssid, password, deviceId } -> prueba, guarda y reinicia.
void handleConnect() {
  enviarCors();

  if (!apServer.hasArg("plain")) {
    apServer.send(400, "application/json", "{\"error\":\"body vacio\"}");
    return;
  }

  JsonDocument doc;
  const DeserializationError err = deserializeJson(doc, apServer.arg("plain"));
  if (err) {
    apServer.send(400, "application/json", "{\"error\":\"json invalido\"}");
    return;
  }

  const String ssid = doc["ssid"] | "";
  const String pass = doc["password"] | "";
  const String id   = doc["deviceId"] | "";

  if (ssid.length() == 0 || id.length() == 0) {
    apServer.send(400, "application/json",
                  "{\"error\":\"ssid y deviceId son obligatorios\"}");
    return;
  }

  Serial.printf("[AP] Probando conexion a '%s' (deviceId=%s)...\n",
                ssid.c_str(), id.c_str());

  // Intentar conectar (el AP sigue vivo por si falla).
  WiFi.begin(ssid.c_str(), pass.c_str());
  const unsigned long inicio = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - inicio < 15000) {
    delay(300);
  }

  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("[AP] No se pudo conectar (revisa clave o senal).");
    apServer.send(200, "application/json",
                  "{\"ok\":false,\"error\":\"no se pudo conectar al WiFi\"}");
    WiFi.disconnect();
    return;
  }

  Serial.print("[AP] Conexion OK, IP: ");
  Serial.println(WiFi.localIP());

  guardarCredenciales(ssid, pass, id);

  JsonDocument res;
  res["ok"] = true;
  res["ip"] = WiFi.localIP().toString();
  res["deviceId"] = id;
  String out;
  serializeJson(res, out);
  apServer.send(200, "application/json", out);

  Serial.println("[AP] Aprovisionado. Reiniciando en 2 s...");
  delay(2000);
  ESP.restart();
}

void handleResetHttp() {
  enviarCors();
  apServer.send(200, "application/json", "{\"ok\":true}");
  delay(300);
  borrarCredenciales();
}

void iniciarModoAP() {
  provisioningMode = true;

  const String apName = String(AP_PREFIX) + sufijoMac();
  WiFi.mode(WIFI_AP_STA);  // AP para el portal + STA para escanear redes
  WiFi.softAP(apName.c_str(), AP_PASSWORD);

  const IPAddress ip = WiFi.softAPIP();
  Serial.println();
  Serial.println("==================================================");
  Serial.println("   MODO APROVISIONAMIENTO (sin WiFi configurado)");
  Serial.println("==================================================");
  Serial.printf("   Red WiFi:  %s\n", apName.c_str());
  Serial.printf("   Clave:     %s\n", AP_PASSWORD);
  Serial.print ("   Portal:    http://");
  Serial.println(ip);
  Serial.println("   La app AquaSave debe conectarse a esta red.");
  Serial.println("==================================================");

  apServer.on("/info", HTTP_GET, handleInfo);
  apServer.on("/scan", HTTP_GET, handleScan);
  apServer.on("/connect", HTTP_POST, handleConnect);
  apServer.on("/reset", HTTP_POST, handleResetHttp);
  apServer.onNotFound(handleCors);  // responde OPTIONS (preflight CORS)
  apServer.enableCORS(true);
  apServer.begin();
}

// Mantener BOOT pulsado 3 s en operacion normal borra credenciales.
void vigilarBotonReset() {
  if (digitalRead(PIN_BOOT) == LOW) {   // pulsado
    if (bootPressedSince == 0) bootPressedSince = millis();
    if (millis() - bootPressedSince >= 3000) {
      Serial.println("[BOOT] Reset de aprovisionamiento solicitado.");
      borrarCredenciales();
    }
  } else {
    bootPressedSince = 0;
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
  Serial.println("[DHT] ERROR: ningun tipo respondio. Revisa cableado.");
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
//  CALIBRACION AUTOAJUSTABLE DEL SENSOR DE SUELO
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
  Serial.printf("[WiFi] Conectando a %s\n", WIFI_SSID.c_str());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(1000);

  WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());

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

  const String clientId = "aquasave-" + DEVICE_ID;
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
      WiFi.begin(WIFI_SSID.c_str(), WIFI_PASSWORD.c_str());
    }
    return;
  }

  if (!mqtt.connected() && millis() - ultimoIntentoMqttMs >= 5000) {
    // La reconexion TLS puede bloquear varios segundos y estirar un pulso;
    // durante un pulso automatico (max 1 min) se pospone. En manual no se
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
  Serial.printf("Device: %s\n", DEVICE_ID.c_str());
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
