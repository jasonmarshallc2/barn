#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal.h>

int lightPin = 6;
unsigned long delayTime = millis();

Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
//const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
const int rs = 7, en = 8, d4 = 9, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  pinMode (lightPin, OUTPUT);
  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);

    
  Serial.begin(9600);
  Serial.println(F("BMP280 Sensor event test"));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
                      "try a different address!"));
    while (1) delay(10);
  }

  /* Default settings from datasheet. */
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  bmp_temp->printSensorDetails();

  displayOn();
}

void loop() {
  sensors_event_t temp_event, pressure_event;
  bmp_temp->getEvent(&temp_event);
  bmp_pressure->getEvent(&pressure_event);
  
  Serial.print(F("Temperature = "));
  Serial.print(temp_event.temperature);
  Serial.println(" *C");

//  Serial.print(F("Pressure = "));
//  Serial.print(pressure_event.pressure);
//  Serial.println(" hPa");

  Serial.println();
lcd.clear();
  lcd.setCursor(1, 0);
  lcd.print("Temp ");
  lcd.print((temp_event.temperature) * 9/5 + 32);

  delay(2000);

  if ((millis() - delayTime) > 10000){
    displayOff();
  }
}

void displayOff(){
  lcd.clear();
  digitalWrite(lightPin, HIGH);
}
void displayOn(){
  digitalWrite(lightPin, LOW);
}
