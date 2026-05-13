// --- Definición de Pines ---
const int pinHumedad = A2;     // Entrada analógica para el sensor de humedad
const int pinTemperatura = A3; // Entrada analógica para el sensor de temperatura
const int pinMotor = 6;        // Salida PWM para el motor

// --- Umbrales de Activación (Ajustables) ---
const float UMBRAL_TEMP = 30.0;   // Temperatura en °C a partir de la cual se enciende el motor
const int UMBRAL_HUMEDAD = 300;   // Valor de humedad por debajo del cual se enciende el motor (0-1023)

void setup() {
  // Configurar el pin del motor como salida
  pinMode(pinMotor, OUTPUT);
  
  // Iniciar la comunicación serial para ver los datos en pantalla
  Serial.begin(9600);
}

void loop() {
  // 1. Leer los valores de los sensores
  int valorHumedad = analogRead(pinHumedad);
  int lecturaTemp = analogRead(pinTemperatura);

  // 2. Convertir la lectura del sensor de temperatura a Grados Celsius
  // (Esta fórmula asume el uso del sensor TMP36 de Tinkercad)
  float voltaje = lecturaTemp * (5.0 / 1024.0);
  float tempCelsius = (voltaje - 0.5) * 100.0;

  // 3. Determinar la velocidad del motor (PWM va de 0 a 255)
  int velocidadMotor = 0; 

  // LÓGICA: Si hace más calor que el umbral O la humedad es más baja que el umbral
  if (tempCelsius > UMBRAL_TEMP || valorHumedad < UMBRAL_HUMEDAD) {
    velocidadMotor = 255; // Encender el motor a máxima potencia
    // Nota: Puedes cambiar 255 por un valor entre 0 y 255 si deseas menos velocidad
  } else {
    velocidadMotor = 0;   // Apagar el motor
  }

  // 4. Enviar la señal al motor
  analogWrite(pinMotor, velocidadMotor);

  // 5. Mostrar los datos en el Monitor Serie de Tinkercad para depuración
  Serial.print("Humedad: ");
  Serial.print(valorHumedad);
  Serial.print(" | Temp: ");
  Serial.print(tempCelsius);
  Serial.print(" C | Motor PWM: ");
  Serial.println(velocidadMotor);

  // Esperar medio segundo antes de la siguiente lectura para dar estabilidad
  delay(500); 
}