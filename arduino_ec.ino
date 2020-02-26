#include "DFRobot_EC.h"
#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <Wire.h>

#define EC_PIN A1
#define ONE_WIRE_BUS 2
#define I2C_ADDRESS 0x07
#define DEBUG 1

float voltage;
float ecValue;
float temperature;
DFRobot_EC ec;

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  // Configure UART
  Serial.begin(9600);

  // Configure I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);

  // Configure DS18B20
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);
  sensors.begin();
  temperature = readTemperature();

  // Configure EC
  ec.begin();
  ec.calibration(voltage, temperature);
}

void loop() {
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {
    timepoint = millis();

    // Temperature sensor (DS18B20)
    temperature = readTemperature();

    // EC sensor
    ecValue = readEC(temperature);

    // Send to UART
#if DEBUG == 1
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.print("^C  EC: ");
    Serial.print(ecValue, 2);
    Serial.println("ms/cm");
#endif
  }
}

void requestEvent()
{
  byte data[] = {
    (byte)temperature,
    (byte)((temperature - (byte)temperature) * 0xFF),
    (byte)ecValue,
    (byte)((ecValue - (byte)ecValue) * 0xFF),
  };
  Wire.write(data, 4);
}

float readEC(float temperature) {
  voltage = analogRead(EC_PIN) / 1024.0 * 5000;
  return ec.readEC(voltage, temperature);
}

float readTemperature() {
  return 25.0;
  sensors.requestTemperatures();
  return sensors.getTempCByIndex(0);
}
