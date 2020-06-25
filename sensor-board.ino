//#include "DFRobot_EC.h"
//#include <DallasTemperature.h>
#include <EEPROM.h>
#include <OneWire.h>
#include <Wire.h>
#include "MeOrion.h"
#include <SoftwareSerial.h>
//#include "GravityTDS.h"



#define VCC_TEMP_PIN 3 //VCC for temp sensor Digital Pin 3
#define GND_TEMP_PIN 4 //GND for temp Digital Pin 4
#define TEMP_PIN 2  //Digital Pin 2
#define EC_PIN A1   //Analog PIN 1
#define VCC_EC_PIN 5 //VCC for EC Digital Pin 5
#define GND_EC_PIN 6 //GND for EC Digital Pin 6
#define PH_PIN A2 //Analog PIN 2
#define PH_SLOPE -39 //Slope of the PH via experiment
#define PH_OFFSET 1118 //From experiment
#define ONE_WIRE_BUS 2
#define I2C_ADDRESS 0x07
#define DEBUG 1
#define WATER_OUT 11
#define WATER_IN 12
//GravityTDS gravityTds;
float voltage;
float tdsValue = 0;
float ecValue = 0;
float phValue;
float phSlope = -39;
float phOffset = 1118;
float temperature = 25;
//DFRobot_EC ec;

OneWire oneWire(ONE_WIRE_BUS);
//DallasTemperature sensors(&oneWire);
MeTemperature myTemp(TEMP_PIN); //Create class myTemp

void setup() {
  // Configure UART
  Serial.begin(9600);

  // Configure I2C
  Wire.begin(I2C_ADDRESS);
  Wire.onRequest(requestEvent);

  // Configure DS18B20 - Temperature Sensor
  pinMode(TEMP_PIN, INPUT);
  pinMode(VCC_TEMP_PIN, OUTPUT);
  digitalWrite(VCC_TEMP_PIN, HIGH);
  pinMode(GND_TEMP_PIN, OUTPUT);
  digitalWrite(GND_TEMP_PIN, LOW);
  myTemp.setpin(TEMP_PIN); //Set the pin to be used for temperature measurement
  //sensors.begin();
  temperature = myTemp.temperature();

  //Water level sensor
  pinMode(WATER_OUT,OUTPUT);
  pinMode(WATER_IN,INPUT);
  digitalWrite(WATER_OUT,HIGH);
  
  // Configure EC
  pinMode(EC_PIN, INPUT);
  pinMode(VCC_EC_PIN, OUTPUT); //Set VCC pin to output
  digitalWrite(VCC_EC_PIN, HIGH); //Set VCC Pin to 5V
  pinMode(GND_EC_PIN, OUTPUT); //Set GND pin to output
  digitalWrite(GND_EC_PIN, LOW); //Set GND Pin to 0V
  //ec.begin();
  //ec.calibration(voltage, temperature);
  
  //Configure PH
  pinMode(PH_PIN,INPUT);
  /*pinMode(VCC_PH_PIN, OUTPUT);
  digitalWrite(VCC_PH_PIN, HIGH);
  pinMode(GND_PH_PIN, OUTPUT);
  digitalWrite(GND_PH_PIN, LOW);*/
}

void loop() {
  static unsigned long timepoint = millis();
  if (millis() - timepoint > 1000U) {
    timepoint = millis();

    // Temperature sensor (DS18B20)
    temperature = myTemp.temperature();

    // EC sensor
    ecValue = readEC(temperature);

    // PH sensor

    phValue = (analogRead(PH_PIN)-phOffset)/phSlope;
    // Send to UART
#if DEBUG == 1
    Serial.print("Temperature: ");
    Serial.print(temperature, 1);
    Serial.print("^C  EC: ");
    Serial.print(ecValue, 2);
    Serial.print("ms/cm  PH: ");
    Serial.print(phValue);
    Serial.println("");
    Serial.print("Water In :");
    Serial.println(digitalRead(WATER_IN));
#endif
  }
}

void requestEvent()
{
  byte data[] = {
    (byte)temperature,                                //Integer part
    (byte)((temperature - (byte)temperature) * 100),  //Decimal part
    (byte)ecValue,
    (byte)((ecValue - (byte)ecValue) * 100),
    (byte)phValue,
    (byte)((phValue - (byte)phValue) * 100),
  };
  #if DEBUG == 1
  Serial.println("Request received, sending data");
  #endif
  Wire.write(data, 6);
}

float readEC(float temperature) {
  
  float analogValue = analogRead(EC_PIN);
  float voltage = analogValue/1024.0*5.0;
  float kValue = 1.0;
  float ecValue=(133.42*voltage*voltage*voltage - 255.86*voltage*voltage + 857.39*voltage)*kValue; //us/cm
  float ecValue25  =  ecValue / (1.0+0.02*(temperature-25.0));  //temperature compensation
  float TdsFactor = 0.5;
  tdsValue = ecValue25 * TdsFactor;
  return ecValue25/1000;
  
}
