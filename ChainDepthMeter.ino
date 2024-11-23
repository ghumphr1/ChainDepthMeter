//Arduino Hall Effect Sensor Chain Depth Project
//by Vizualart.

#include <LiquidCrystal.h>
#include <EEPROM.h>
#include "DHT_Async.h"

#define DHT_SENSOR_TYPE DHT_TYPE_22

LiquidCrystal lcd(12,11,7,6,5,4);

const int hallPin = 2;                    // Timing input from Capstan
static const int DHT_SENSOR_PIN = 3;     // Temp/Humidity
const int switchUpPin = 8;
const int switchDownPin = 9;
const int saveMemoryPin = 10;

int eepromAddr = 0;
int metres = 0;                    // TO DO - conversion of revs to chain length

unsigned long timer = 0;
unsigned long interval = 500;

volatile int rev = 0;               //  revolutions of capstan

boolean switchStateDown = 0;        // current state of the down button
boolean lastSwitchStateDown = 0;    // previous state of the down button
boolean switchStateUp = 0;
boolean lastSwitchStateUp = 0;
boolean switchStateSave = 0;
boolean lastswitchStateSave = 0;

DHT_Async dht_sensor(DHT_SENSOR_PIN, DHT_SENSOR_TYPE);

const uint8_t clrArrow[] = {        //  clear Arrow from display
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
        0b00000,
    };

const uint8_t upArrow[] = {         //  arrow up
        0b00100,
        0b01110,
        0b10101,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00000,
    };

const uint8_t downArrow[] = {       //  arrow down
        0b00000,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b10101,
        0b01110,
        0b00100,
    };

void countChainDown() {
  rev++;
  Serial.print(rev);
  Serial.println("detect");
  lcd.setCursor(9,3);
  lcd.print(rev);
  delay(100);
}

void countChainUp() {
  if (rev>0){
    rev--;                          //stop count going below zero on chain up
  }
  Serial.print(rev);
  Serial.println("detect");
  lcd.setCursor(9,3);
  lcd.print(rev);
  delay(100);
}

void clearEeprom(){                                       // Clear EEPROM memory
  for (int i = 0 ; i < EEPROM.length() ; i++) {
    EEPROM.write(i, 0);
  }
}

void setup() {
  clearEeprom();
  rev = EEPROM.read(eepromAddr);

  lcd.createChar(1, downArrow);
  lcd.createChar(2, upArrow);
  lcd.createChar(3, clrArrow);

  lcd.begin(20,4);
  lcd.setCursor(0,0);
  lcd.print("Initialising.");
  delay(500);
  lcd.clear();
  lcd.print("Initialising..");
  delay(500);
  lcd.clear();
  lcd.print("Initialising...");
  delay(500);
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.print("VizualArt");
  lcd.setCursor(0,1);
  lcd.print("     Chain Depth");
  lcd.setCursor(9,3);
  lcd.print(rev);
  lcd.setCursor(14,3);
  lcd.print("Metre");
  delay(1000);

  pinMode(hallPin, INPUT_PULLUP);
  pinMode(switchUpPin, INPUT_PULLUP);
  pinMode(switchDownPin, INPUT_PULLUP);
  pinMode(saveMemoryPin, INPUT_PULLUP);
  Serial.begin(115200);
  timer = millis();
}

void loop() {

    float temperature;
    float humidity;

   if (millis() - timer >= interval){
      timer = millis();
            
      switchStateDown = digitalRead(switchDownPin);         // read the pushbutton input pin:
      
      if (switchStateDown != lastSwitchStateDown)           // compare the buttonState to its previous state
      {         
         if (switchStateDown == LOW)                        // if the state has changed, increment the counter
         {
            // if the current state is LOW then the button went from off to on:
            lcd.setCursor(0,3);
            lcd.print((char)0x01);
            Serial.println("down on");
            attachInterrupt(digitalPinToInterrupt(hallPin), countChainDown, FALLING);
         }
         else
         {
            // if the current state is HIGH then the button went from on to off:
            lcd.setCursor(0,3);
            lcd.print((char)0x03);
            Serial.println("down off");
         }
        
          lastSwitchStateDown = switchStateDown;       // save the current state as the last state, for next time through the loop
      }

      delay(200);
      
      switchStateUp = digitalRead(switchUpPin);

      if (switchStateUp != lastSwitchStateUp)
      {
         if (switchStateUp == LOW)
         {
            lcd.setCursor(0,3);
            lcd.print((char)0x02);
            Serial.println("up on");
            attachInterrupt(digitalPinToInterrupt(hallPin), countChainUp, FALLING);
         }
         else
         {
            lcd.setCursor(0,3);
            lcd.print((char)0x03);
            Serial.println("up off");
         }
          lastSwitchStateUp = switchStateUp;
      }

      delay(200);

      switchStateSave = digitalRead(saveMemoryPin);         // read the pushbutton input pin:
      
      if (switchStateSave != lastswitchStateSave)           // compare the buttonState to its previous state
      {         
         if (switchStateSave == LOW)                        // if the state has changed, increment the counter
         {
            // if the current state is LOW then the button went from off to on:
            lcd.setCursor(0,3);
            lcd.print((char)0x01);
            Serial.println("save on");
            Serial.print("RevSave ");
            Serial.println(rev);
            EEPROM.update(eepromAddr, rev);                          //save counter to eeprom
         }
          else
         {
            // if the current state is HIGH then the button went from on to off:
            lcd.setCursor(0,3);
            lcd.print((char)0x03);
            Serial.println("save off");
         }
        
          lastswitchStateSave = switchStateSave;       // save the current state as the last state, for next time through the loop
      }

//      if (measure_environment(&temperature, &humidity)) {
//        Serial.print("T = ");
//        Serial.print(temperature, 1);
//        Serial.print(" deg. C, H = ");
//        Serial.print(humidity, 1);
//        Serial.println("%");
//      }

      delay(200); 
  }
}
