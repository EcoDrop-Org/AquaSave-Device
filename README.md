# AquaSave-Device

Firmware del dispositivo IoT de AquaSave (ESP32). Lee la humedad del suelo y
el clima ambiente, controla la bomba de riego de forma autónoma (por pulsos)
y se conecta al resto del sistema vía MQTT sobre TLS.

```
ESP32 ──MQTT/TLS──► HiveMQ Cloud ──► AquaSave EdgeAPI ──HTTP──► Backend ◄── App Flutter
      ◄──MQTT──── (open-valve / close-valve / pause-device / resume-device)
```

## Hardware

| Componente | Conexión |
|---|---|
| HW-390 (humedad de suelo, capacitivo) | AOUT → GPIO34, VCC → 3.3V, GND → GND |
| DHT22 (o DHT11) temp/humedad de aire | DATA → GPIO16, VCC → 3.3V, GND → GND |
| Módulo relé 1 canal | IN → GPIO26, VCC → 5V (VIN), GND → GND |
| Bomba 5V | COM relé → +5V fuente externa, NO relé → rojo bomba, negro bomba → GND fuente |
| Diodo 1N4007 | En paralelo con la bomba (franja/cátodo al cable rojo) |

> GND de la fuente externa y GND del ESP32 deben estar unidos.

## Configuración antes de flashear (WiFi fijo)

El dispositivo usa **WiFi fijo**: las credenciales van en el código y se
conecta solo al encender (ya no hay modo de aprovisionamiento por AP).
Editar el bloque `Configuracion` al inicio de `AquaSave.ino`:

1. `WIFI_SSID` / `WIFI_PASSWORD` — red WiFi de **2.4 GHz**.
2. `DEVICE_ID` — ID del huerto creado en la app AquaSave
   (Dispositivos → tarjeta del huerto → **Ver detalles** →
   **ID del dispositivo** → copiar).
3. `MQTT_HOST` / credenciales — ya apuntan al HiveMQ Cloud del proyecto.

## Librerías (Arduino IDE → Library Manager)

- **DHT sensor library** (Adafruit) + **Adafruit Unified Sensor**
- **PubSubClient** (Nick O'Leary)
- **ArduinoJson** (Benoît Blanchon, v7)

Placa: `ESP32 Dev Module`.

## Cómo funciona

- **Riego automático por pulsos:** si el suelo ≤ 40 %, riega un pulso de 3 s
  y descansa 5 min para que el agua se absorba; repite hasta llegar al 70 %.
  Con calor (≥ 30 °C) y aire seco (≤ 40 %) también riega en zona intermedia.
  Funciona aunque no haya WiFi.
- **Fail-safe del sensor de suelo:** si el ADC lee fuera de rango físico
  (sensor suelto/roto) no hay riego automático.
- **Calibración autoajustable:** la ventana seco/mojado del HW-390 se ajusta
  con los extremos observados y se guarda en NVS.
- **Modo MANUAL:** `open-valve` fuerza la bomba ON hasta `close-valve`,
  10 min máximo o suelo saturado (≥ 90 %).
- **Pausa remota:** `pause-device` bloquea todo riego (persiste tras
  reinicios) hasta `resume-device`; la telemetría sigue reportando.
- **Telemetría cada 5 s** → `aquasave/devices/<id>/telemetry`:
  `{ soilMoisturePct, adcRaw, soilSensorOk, temperatureC, humidityPct,
  pumpOn, flowRateLMin }`. `flowRateLMin` es el caudal nominal de la bomba
  (no hay sensor de flujo); el backend lo usa para estimar litros.
- **Estado** → `aquasave/devices/<id>/status` (retained + Last Will
  `offline`).
- **Comandos** ← `aquasave/devices/<id>/commands`; cada comando se confirma
  en `aquasave/devices/<id>/commands/ack`.

## DHT22: verificación y solución de problemas

El firmware **prueba primero DHT22 y luego DHT11** al arrancar y se queda con
el que responda con datos coherentes; respeta los ≥ 2 s entre lecturas que
exige el sensor y descarta valores absurdos. Si tras 10 lecturas seguidas
falla, vuelve a detectar el tipo automáticamente.

En el monitor serie (115200) debe verse al arrancar:

```
[DHT] Detectando sensor (DHT22 / DHT11)...
[DHT] Detectado DHT22 (24.3 C, 61.0 %)
```

Si sale `SIN LECTURA`:

1. VCC a **3.3V** y GND común con el ESP32.
2. Sensor de 4 pines "pelado": necesita **pull-up de 10k entre VCC y DATA**
   (los módulos de 3 pines ya la traen).
3. Cables dupont firmes y cortos; un falso contacto produce NaN.
4. Placas ESP32-**WROVER**: GPIO16 lo usa la PSRAM — mover DATA a GPIO4 y
   cambiar `PIN_DHT`.

Si el DHT no responde, el riego por humedad de suelo sigue funcionando y la
telemetría se envía sin temperatura/humedad de aire (el backend reutiliza la
última temperatura conocida).
