#include "../lib/CircularBuffer.cpp"
#include "../lib/LCD_I2C.cpp"
#include "Arduino.h"
#include "esp_task_wdt.h"
#include <Adafruit_BME280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <string>

using namespace std;

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

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_RESET -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS                                                         \
  0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

LCD_I2C lcd(0x3F, 20, 4);

bool firstTime = false;

void setupLCD() {
  lcd.init();
  lcd.backlight();
}

void setupOLED() {}

void resetBuffers() {
  for (int x = 0; x < bufferSize; x++) {
    bmeTemperaturebuffer.pushElement(0);
    bmeHumidityBuffer.pushElement(0);
    bmePressureBuffer.pushElement(0);
    bmeAltitudeBuffer.pushElement(0);
  }
}

float roundToOneDecima(float input) { return round(input * 10) / 10; };

void loop0() { // read

  if (!firstTime) {

    lcd.firstLine("Inicializando...");
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Inicializando");
    display.display();

    for (int i = 5; i > 0; i--) {

      String text = "Aguarde " + String(i) + "s";

      lcd.secondLine(text);

      delay(1000);
    }

    firstTime = true;
  }

  float bmeTemperature = bmeTemperaturebuffer.averageLast(bufferSize);
  float bmeHumidity = bmeHumidityBuffer.averageLast(bufferSize);
  float bmePressure = bmePressureBuffer.averageLast(bufferSize);
  float bmeAltitude = bmeAltitudeBuffer.averageLast(bufferSize);

  bmePressure = (round(bmePressure * 10) / 10);
  bmeAltitude = (round(bmeAltitude * 10) / 10);

  String firstLine =
      "T " + String(bmeTemperature) + "C H " + String(bmeHumidity);
  // +"%";
  String secondLine =
      "P " + String(bmePressure, 1) + "P A " + String(bmeAltitude, 1);
  // +"m";

  lcd.firstLine(firstLine);
  lcd.secondLine(secondLine);

  // delay(751);

  display.clearDisplay();

  display.setTextSize(1.9);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Temperat: " + String(bmeTemperature) + " C");
  display.println("Humidity: " + String(bmeHumidity) + " %");
  display.println("Pressure: " + String(bmePressure, 1) + " hPa");
  display.println("Altitude: " + String(bmeAltitude, 1) + " m");

  // display.setTextColor(SSD1306_BLACK, SSD1306_WHITE); // Draw 'inverse' text
  // display.println(3.141592);

  // display.setTextSize(2); // Draw 2X-scale text
  // display.setTextColor(SSD1306_WHITE);
  // display.print(F("0x"));
  // display.println(0xDEADBEEF, HEX);

  display.display();

  delay(751);

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
  bmeTemperaturebuffer.pushElement(roundToOneDecima(bme.readTemperature()));
  bmeHumidityBuffer.pushElement(bme.readHumidity());
  bmePressureBuffer.pushElement(bme.readPressure() / 100.0F);
  bmeAltitudeBuffer.pushElement(bme.readAltitude(SEALEVELPRESSURE_HPA));
  delay(47);
}

void runTask(int cpu) {
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

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ; // Don't proceed, loop forever
  }

  display.clearDisplay();
  display.display();

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