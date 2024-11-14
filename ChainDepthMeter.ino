//Arduino Hall Effect Sensor Chain Depth Project
//by Vizualart.

#include<LiquidCrystal.h>

LiquidCrystal lcd(12,10,7,6,5,4);

const int hallPin = 2;
const int switchUpPin = 3;
const int metres = 0;
int direction = 1;

volatile int rev = 0;

const uint8_t upArrow[] = {
        0b00100,
        0b01110,
        0b10101,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b00000,
    };

const uint8_t downArrow[] = {
        0b00000,
        0b00100,
        0b00100,
        0b00100,
        0b00100,
        0b10101,
        0b01110,
        0b00100,
    };    

void count(){}    

void countDown() {
  rev++;
  Serial.print(rev);
  Serial.println("detect");
  lcd.setCursor(0,3);
  lcd.print((char)0x01);
  lcd.setCursor(9,3);
  lcd.print(rev);
  delay(100);
}

void countUp() {
  rev--;
  Serial.print(rev);
  Serial.println("detect");
  lcd.setCursor(0,3);
  lcd.print((char)0x02);  
  lcd.setCursor(9,3);
  lcd.print(rev);
  delay(100);
}
/home/ghumphr1/workspace/Arduino/ChainDepthCounter/linkCounter/Transistor as 5V buffer.JPG
  lcd.createChar(2, upArrow);
  Serial.begin(115200);
  lcd.begin(20,4);
  lcd.setCursor(0,0);
  lcd.print("Initialising.");
  delay(1000);
  lcd.clear();
  lcd.print("Initialising..");
  delay(1000);
  lcd.clear();
  lcd.print("Initialising...");
  delay(1000);
  lcd.clear();
  lcd.setCursor(6,0);
  lcd.print("VizualArt");
  lcd.setCursor(0,1);
  lcd.print("     Chain Depth");
  lcd.setCursor(14,3);
  lcd.print("Metre");
  delay(2000);
  pinMode(hallPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(hallPin), count, FALLING);
  attachInterrupt(digitalPinToInterrupt(switchUpPin), direction, LOW);
}

void loop() {

}
