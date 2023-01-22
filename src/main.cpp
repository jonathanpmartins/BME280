#include "../lib/CircularBuffer.cpp"
#include "../lib/LCD_I2C.cpp"
#include "Arduino.h"
#include "esp_task_wdt.h"
#include <Adafruit_BME280.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

TaskHandle_t CPU0;
TaskHandle_t CPU1;

#define SEALEVELPRESSURE_HPA (1013.25)

#define DHT_11_PIN 4
#define DHT_22_PIN 0

Adafruit_BME280 bme;

int bufferSize = 127;

CircularBuffer bmeTemperaturebuffer(bufferSize);
CircularBuffer bmeHumidityBuffer(bufferSize);
CircularBuffer bmePressureBuffer(bufferSize);
CircularBuffer bmeAltitudeBuffer(bufferSize);

LCD_I2C lcd(0x3F, 20, 4);

bool firstTime = false;

void setupLCD() {
  lcd.init();
  lcd.backlight();
}

void resetBuffers() {
  for (int x = 0; x < bufferSize; x++) {
    bmeTemperaturebuffer.pushElement(0);
    bmeHumidityBuffer.pushElement(0);
    bmePressureBuffer.pushElement(0);
    bmeAltitudeBuffer.pushElement(0);
  }
}

void loop0() { // read

  if (!firstTime) {

    lcd.firstLine("Inicializando");

    delay(1000);

    firstTime = true;
  }

  float bmeTemperature = bmeTemperaturebuffer.averageLast(bufferSize);
  float bmeHumidity = bmeHumidityBuffer.averageLast(bufferSize);
  float bmePressure = bmePressureBuffer.averageLast(bufferSize);
  float bmeAltitude = bmeAltitudeBuffer.averageLast(bufferSize);

  String firstLine =
      "T " + String(bmeTemperature) + "C H " + String(bmeHumidity);
  // +"%";
  String secondLine = "P " + String(bmePressure) + " A " + String(bmeAltitude);
  // +"m";

  lcd.firstLine(firstLine);
  lcd.secondLine(secondLine);

  delay(250);

  // Serial.print("BME280  - Temperature = ");
  // Serial.print(bmeTemperaturebuffer.averageLast(bufferSize));
  // Serial.print(" °C, Humidity = ");
  // Serial.print(bmeHumidityBuffer.averageLast(bufferSize));
  // Serial.print("% - ");

  // Serial.print("Pressure = ");
  // Serial.print(bmePressureBuffer.averageLast(bufferSize));
  // Serial.print(" hPa - ");

  // Serial.print("Approx. Altitude = ");
  // Serial.print(bmeAltitudeBuffer.averageLast(bufferSize));
  // Serial.print(" m");

  // Serial.println();

  // delay(750);
}

void loop1() { // write
  bmeTemperaturebuffer.pushElement(bme.readTemperature());
  bmeHumidityBuffer.pushElement(bme.readHumidity());
  bmePressureBuffer.pushElement(bme.readPressure() / 100.0F);
  bmeAltitudeBuffer.pushElement(bme.readAltitude(SEALEVELPRESSURE_HPA));
  delay(5);
}

void runTask(int cpu) {
  Serial.println(cpu);
  while (true) {
    if (cpu == 0) {
      loop0();
    }
    if (cpu == 1) {
      loop1();
    }

    esp_task_wdt_reset();
  }
}

void taskCPU0(void *parameters) { runTask(0); }
void taskCPU1(void *parameters) { runTask(1); }

void setup() {
  delay(250);

  Serial.begin(115200);
  while (!Serial)
    ;

  setupLCD();
  resetBuffers();

  Serial.println(F("BME280 test"));
  if (bme.begin(0x76) == false) {
    Serial.println("Could not find a valid BME280 sensor, check wiring!");
    while (1)
      ;
  }

  const int stackSize = 32768;

  xTaskCreatePinnedToCore(taskCPU0, "CPU0", stackSize, NULL, 0, &CPU0, 0);
  xTaskCreatePinnedToCore(taskCPU1, "CPU1", stackSize, NULL, 1, &CPU1, 1);

  delay(250);
}

void loop() { vTaskDelete(NULL); }