#include <DHT.h>

#define PIN_SENSOR_SUELO 34
#define PIN_DHT 16
#define PIN_RELE 26

#define DHTTYPE DHT22

DHT dht(PIN_DHT, DHTTYPE);

const int SUELO_SECO = 2000;
const int SUELO_HUMEDO = 1500;

const float TEMP_ALTA = 30.0;
const float HUMEDAD_AIRE_BAJA = 40.0;

const unsigned long INTERVALO = 5000;

bool bombaEncendida = false;
unsigned long tiempoAnterior = 0;

float ultimaTemperatura = 0;
float ultimaHumedadAire = 0;
bool hayLecturaDHT = false;

void setup() {
  Serial.begin(115200);
  delay(2000);

  pinMode(PIN_RELE, OUTPUT);
  digitalWrite(PIN_RELE, LOW);  // LOW = bomba apagada

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_SENSOR_SUELO, ADC_11db);

  dht.begin();

  Serial.println("====================================");
  Serial.println(" SISTEMA DE RIEGO AUTOMATICO ESP32");
  Serial.println("====================================");
  Serial.println("HW-390 AOUT -> GPIO34");
  Serial.println("DHT22 OUT   -> GPIO16");
  Serial.println("RELE        -> GPIO26");
  Serial.println("HIGH = BOMBA ON");
  Serial.println("LOW  = BOMBA OFF");
  Serial.println("====================================");
}

void loop() {
  unsigned long tiempoActual = millis();

  if (tiempoActual - tiempoAnterior >= INTERVALO) {
    tiempoAnterior = tiempoActual;

    int humedadSuelo = analogRead(PIN_SENSOR_SUELO);

    float temperatura;
    float humedadAire;
    bool dhtOK = leerDHTSeguro(temperatura, humedadAire);

    if (dhtOK) {
      ultimaTemperatura = temperatura;
      ultimaHumedadAire = humedadAire;
      hayLecturaDHT = true;
    }

    bool sueloSeco = humedadSuelo >= SUELO_SECO;
    bool sueloHumedo = humedadSuelo <= SUELO_HUMEDO;

    bool ambienteCalienteSeco = false;

    if (hayLecturaDHT) {
      ambienteCalienteSeco = ultimaTemperatura >= TEMP_ALTA && ultimaHumedadAire <= HUMEDAD_AIRE_BAJA;
    }

    if (sueloSeco) {
      bombaEncendida = true;
    }

    if (!sueloHumedo && ambienteCalienteSeco) {
      bombaEncendida = true;
    }

    if (sueloHumedo) {
      bombaEncendida = false;
    }

    digitalWrite(PIN_RELE, bombaEncendida ? HIGH : LOW);

    Serial.println();
    Serial.println("========== NUEVA LECTURA ==========");

    Serial.print("ADC suelo: ");
    Serial.println(humedadSuelo);

    Serial.print("Estado suelo: ");
    if (sueloSeco) {
      Serial.println("SECO");
    } else if (sueloHumedo) {
      Serial.println("HUMEDO");
    } else {
      Serial.println("INTERMEDIO");
    }

    Serial.print("Temperatura DHT22: ");
    if (hayLecturaDHT) {
      Serial.print(ultimaTemperatura);
      Serial.println(" C");
    } else {
      Serial.println("SIN LECTURA");
    }

    Serial.print("Humedad aire DHT22: ");
    if (hayLecturaDHT) {
      Serial.print(ultimaHumedadAire);
      Serial.println(" %");
    } else {
      Serial.println("SIN LECTURA");
    }

    Serial.print("Lectura DHT actual: ");
    Serial.println(dhtOK ? "OK" : "FALLO, usando ultima lectura");

    Serial.print("Ambiente caliente/seco: ");
    Serial.println(ambienteCalienteSeco ? "SI" : "NO");

    Serial.print("GPIO26 enviado: ");
    Serial.println(bombaEncendida ? "HIGH" : "LOW");

    Serial.print("Estado bomba: ");
    Serial.println(bombaEncendida ? "ENCENDIDA" : "APAGADA");

    Serial.println("===================================");
  }
}

bool leerDHTSeguro(float &temperatura, float &humedadAire) {
  for (int intento = 1; intento <= 3; intento++) {
    temperatura = dht.readTemperature();
    humedadAire = dht.readHumidity();

    if (!isnan(temperatura) && !isnan(humedadAire)) {
      return true;
    }

    delay(300);
  }

  return false;
}
