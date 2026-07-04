# AquaSave-Device

Firmware del dispositivo IoT de AquaSave (ESP32). Lee la humedad del suelo y
el clima ambiente, controla la bomba de riego de forma autónoma y se conecta
al resto del sistema vía MQTT.

```
ESP32 ──MQTT──► Mosquitto ──► AquaSave EdgeAPI ──HTTP──► AquaSave Backend ◄── App Flutter
      ◄─MQTT── (comandos open-valve / close-valve)
```

## Hardware

| Componente | Conexión |
|---|---|
| HW-390 (humedad de suelo, capacitivo) | AOUT → GPIO34, VCC → 3.3V, GND → GND |
| DHT11 **o** DHT22 (temp/humedad aire) | DATA → GPIO16, VCC → 3.3V, GND → GND |
| Módulo relé 1 canal | IN → GPIO26, VCC → 5V (VIN), GND → GND |
| Bomba 5V | COM relé → +5V fuente externa, NO relé → rojo bomba, negro bomba → GND fuente |
| Diodo 1N4007 | En paralelo con la bomba (franja/cátodo al cable rojo) |

> GND de la fuente externa y GND del ESP32 deben estar unidos.

## Configuración antes de flashear

Editar el bloque `Configuracion` al inicio de `AquaSave.ino`:

1. `WIFI_SSID` / `WIFI_PASSWORD` — red WiFi de **2.4 GHz**.
2. `MQTT_HOST` — IP de la máquina donde corre Mosquitto (el EdgeAPI).
3. `DEVICE_ID` — ID del dispositivo creado en la app AquaSave
   (Dispositivos → tarjeta del dispositivo → **ID del dispositivo** → copiar).

## Librerías (Arduino IDE → Library Manager)

- **DHT sensor library** (Adafruit) + **Adafruit Unified Sensor**
- **PubSubClient** (Nick O'Leary)
- **ArduinoJson** (Benoît Blanchon, v7)

Placa: `ESP32 Dev Module`.

## Cómo funciona

- **Modo AUTO (por defecto):** suelo ≤ 35 % → bomba ON; suelo ≥ 70 % → bomba
  OFF. Si hace calor (≥ 30 °C) y el aire está seco (≤ 40 %), riega también en
  zona intermedia. Funciona aunque no haya WiFi.
- **Modo MANUAL:** al recibir `open-valve` desde la app la bomba se fuerza ON
  hasta recibir `close-valve`, cumplir 10 min o saturar el suelo (≥ 90 %).
- **Telemetría cada 5 s** → `aquasave/devices/<id>/telemetry`:
  `{ soilMoisturePct, temperatureC, humidityPct, pumpOn, flowRateLMin }`.
  `flowRateLMin` es el caudal nominal de la bomba (no hay sensor de flujo);
  el backend lo usa para estimar litros consumidos.
- **Estado** → `aquasave/devices/<id>/status` (retained + Last Will
  `offline` si el ESP32 pierde conexión).
- **Comandos** ← `aquasave/devices/<id>/commands`; cada comando se confirma
  en `aquasave/devices/<id>/commands/ack`.

## Solución de problemas del DHT (t:err / h:err)

El firmware autodetecta si el sensor es DHT11 o DHT22 al arrancar (el error
más común es declarar un tipo que no coincide con el sensor físico). Si aun
así no responde:

1. VCC a **3.3V** y GND común con el ESP32.
2. Sensor de 4 pines "pelado": necesita **pull-up de 10k entre VCC y DATA**
   (los módulos de 3 pines ya la traen).
3. No leer más rápido que cada 2 s (el firmware ya lo respeta).
4. Cables dupont firmes y cortos; un falso contacto produce NaN.
5. Placas ESP32-**WROVER**: GPIO16 lo usa la PSRAM — mover DATA a GPIO4 y
   cambiar `PIN_DHT`.

Si el DHT no responde, el riego por humedad de suelo sigue funcionando y la
telemetría se envía sin temperatura/humedad de aire.

## Calibración del sensor de suelo

En el monitor serie (115200 baudios) se imprime el valor ADC crudo. Con el
sensor al aire ajusta `ADC_SECO_AIRE` (~2600) y sumergido en agua
`ADC_MOJADO` (~1100) si tus valores difieren.
