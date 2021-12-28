/*
 * This is a thermostat for a remote building where the ability to turn off power is required
 * It is controlled by a main power switch as well as a pot to adjust the set temp.
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_BMP280.h>
#include <LiquidCrystal.h>

int setTempPin = A3;
int lightPin = 6;
int furniceRelay = 5;
int powerSwitch = 4;
//unsigned long delayTime = millis();
int setTemp = 0;
bool systemPower = false;
int currentTemp = 0;
sensors_event_t temp_event, pressure_event;
  
Adafruit_BMP280 bmp; // use I2C interface
Adafruit_Sensor *bmp_temp = bmp.getTemperatureSensor();
//Adafruit_Sensor *bmp_pressure = bmp.getPressureSensor();

// initialize the library by associating any needed LCD interface pin
// with the arduino pin number it is connected to
//const int rs = 2, en = 3, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
const int rs = 7, en = 8, d4 = 9, d5 = 11, d6 = 12, d7 = 13;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  pinMode (lightPin, OUTPUT); //disables lcd backlight when not in use
  pinMode (furniceRelay, OUTPUT);
  pinMode (setTempPin, INPUT); //connect to pot that adjusts temp
  pinMode (powerSwitch, INPUT); //connect to main power switch
  digitalWrite(powerSwitch, LOW);
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

  furniceOff();
  displayOff();
}

void loop() {
// Toogle power switch
if (digitalRead(powerSwitch) == HIGH){
  delay(300);
  if (systemPower){
    systemPower = false;
    furniceOff();
    displayOff();
    Serial.println("Power off");
  } else {
    systemPower = true;
    displayOn();
    Serial.println("Power on");
  }
}

if (systemPower){
  bmp_temp->getEvent(&temp_event);
  currentTemp = (temp_event.temperature) * 9/5 + 32;
//  Serial.print(F("Temperature = "));
//  Serial.print(currentTemp);
//  Serial.println(" *F");
  printCurrentTemp();

  int temp = analogRead(setTempPin);
//  setTemp = (temp * 0.0488) + 40; // get about 45 degree total range 50/1024(pot range)
  setTemp = (temp * 0.0588) + 40; // get about 45 degree total range 60/1024(pot range)
  lcd.setCursor(1, 1);
  lcd.print("Set  ");
  lcd.print(setTemp);
  
  if ((setTemp - currentTemp) > 0){
    furniceOn();
  } else if ((setTemp - currentTemp) < -2) {
    furniceOff();
  }
  
  delay(50);
}
}

void furniceOn(){
  digitalWrite(furniceRelay, HIGH);
  Serial.println("Furnice ON");
  printCurrentTemp();
  lcd.setCursor(14, 0);
  lcd.print("ON");
}
void furniceOff(){
  digitalWrite(furniceRelay, LOW);
  Serial.println("Furnice OFF");
  lcd.setCursor(14, 0);
  lcd.print("  ");
}
void displayOff(){ // The display I'm using does not allow a shutoff mode, so just clear it
  lcd.clear();
  digitalWrite(lightPin, HIGH);
  lcd.setCursor(0, 0);
  lcd.print("                ");
  lcd.setCursor(0, 1);
  lcd.print("                ");
}
void displayOn(){
  digitalWrite(lightPin, LOW);
}
void printCurrentTemp(){
  bmp_temp->getEvent(&temp_event);
  currentTemp = (temp_event.temperature) * 9/5 + 32;
  lcd.setCursor(1, 0);
  lcd.print("Temp ");
  lcd.print(currentTemp);
}
