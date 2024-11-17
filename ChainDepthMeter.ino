//Arduino Hall Effect Sensor Chain Depth Project
//by Vizualart.

#include<LiquidCrystal.h>

LiquidCrystal lcd(12,10,7,6,5,4);

const int hallPin = 2;
const int switchUpPin = 8;
const int switchDownPin = 9;
int metres = 0;                     // TO DO

volatile int rev = 0;

boolean switchStateDown = 0;        // current state of the down button
boolean lastswitchStateDown = 0;    // previous state of the down button

boolean switchStateUp = 0;          //  current state of the up button
boolean lastswitchStateUp = 0;      //  previous state of the up button

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
  while (rev<0){
    rev--;
  }
  Serial.print(rev);
  Serial.println("detect");
  lcd.setCursor(9,3);
  lcd.print(rev);
  delay(100);
}

void setup() {
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
  Serial.begin(115200);
  
}

void loop() {

  static unsigned long timer = 0;
   unsigned long interval = 20;
   if (millis() - timer >= interval)
   {
      timer = millis();
      
      // read the pushbutton input pin:
      switchStateDown = digitalRead(switchDownPin);
      // compare the buttonState to its previous state
      if (switchStateDown != lastswitchStateDown)
      {
         // if the state has changed, increment the counter
         if (switchStateDown == LOW)
         {
            // if the current state is LOW then the button went from off to on:
            lcd.setCursor(0,3);
            lcd.print((char)0x01);
            Serial.println("on");
            attachInterrupt(digitalPinToInterrupt(hallPin), countChainDown, FALLING);
         }
         else
         {
            // if the current state is HIGH then the button went from on to off:
            lcd.setCursor(0,3);
            lcd.print((char)0x03);
            Serial.println("off");
         }
          // save the current state as the last state, for next time through the loop
          lastswitchStateDown = switchStateDown;
      }

           
      // read the pushbutton input pin:
      switchStateUp = digitalRead(switchUpPin);
      // compare the buttonState to its previous state
      if (switchStateUp != lastswitchStateUp)
      {
         // if the state has changed, increment the counter
         if (switchStateUp == LOW)
         {
            // if the current state is LOW then the button went from off to on:
            lcd.setCursor(0,3);
            lcd.print((char)0x02);
            Serial.println("on");
            attachInterrupt(digitalPinToInterrupt(hallPin), countChainUp, FALLING);
         }
         else
         {
            // if the current state is HIGH then the button went from on to off:
            lcd.setCursor(0,3);
            lcd.print((char)0x03);
            Serial.println("off");
         }
          // save the current state as the last state, for next time through the loop
          lastswitchStateUp = switchStateUp;
      }
  }


}

//   attachInterrupt(digitalPinToInterrupt(hallPin), count, FALLING);
