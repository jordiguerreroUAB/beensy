//#include <Wire.h>
#include <Servo.h>
#include <Arduino.h>
#include <TinyMPU6050.h>

#include <SPI.h>
#include <Wire.h>

#include <ACROBOTIC_SSD1306.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

#define OLED_MOSI   11//9
#define OLED_CLK   13
#define OLED_DC    37
#define OLED_CS    33//39
#define OLED_RESET 38
Adafruit_SSD1306 display1(SCREEN_WIDTH, SCREEN_HEIGHT,
  OLED_MOSI, OLED_CLK, OLED_DC, OLED_RESET, OLED_CS);

Adafruit_SSD1306 display2(SCREEN_WIDTH, 32, &Wire, -1);  

Servo servo1;
Servo servo2;
Servo servo3;

MPU6050 mpu (Wire);

const int led1 = 35;
const int led2 = 29;
const int led3 = 30;
const int button1 = 24;
const int button2 = 25;
const int button3 = 26;
const int buzzer = 17;//27;
const int pot = 39;
//const int pot = 17;

int button1State = 0;
int button2State = 0;
int button3State = 0;

int servo1Pos = 0;
int servo2Pos = 0;
int servo3Pos = 0;

int potVal;
int oldPotVal;

void setup() {
  if(!display1.begin(SSD1306_SWITCHCAPVCC)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  if(!display2.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for(;;);
  }

  servo1.attach(2);
  servo2.attach(5);
  servo3.attach(6);

  mpu.Initialize();

  Wire.begin();  
  display2.clearDisplay();

  potVal = analogRead(pot);
  
  display2.setTextSize(1);
  display2.setTextColor(WHITE);
  display2.setCursor(0, 0);
  display2.println("HELLO WORLD!");
  display2.setCursor(0,10);
  display2.println("Potentiometer: ");
  display2.setCursor(0,20);
  display2.println(potVal);
  display2.display();

  servo1.write(potVal/8);
  servo2.write(potVal/8);
  servo3.write(potVal/8);
  
  
  oldPotVal = potVal;

  

  Serial.begin(9600);
  Serial3.begin(9600);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);
  pinMode(led3, OUTPUT);
  pinMode(button1, INPUT);
}

void loop() {
  potVal = analogRead(pot);

  if (oldPotVal != potVal) {
    display2.clearDisplay();    
    display2.setCursor(0, 0);
    display2.println("HELLO WORLD!");
    display2.setCursor(0,10);
    display2.println("Potentiometer: ");
    display2.setCursor(0,20);
    display2.println(potVal);
    display2.display();   

    servo1.write(potVal/8);
    servo2.write(potVal/8);
    servo3.write(potVal/8);
    analogWrite(led1, potVal/4);
  }
  oldPotVal = potVal;
  
  button1State = digitalRead(button1);
  Serial3.println(potVal);
//  Serial.println(potVal);

  mpu.Execute();
  Serial.print("X= ");
  Serial.print(mpu.GetAngX());
  Serial.print(" / Y= ");
  Serial.print(mpu.GetAngY());
  Serial.print(" / Z= ");
  Serial.println(mpu.GetAngZ());
  
  display1.clearDisplay();

  display1.setTextSize(1);
  display1.setTextColor(SSD1306_WHITE);
  display1.setCursor(0,0);
  display1.print("X=");
  display1.print(mpu.GetAngX());
  display1.print("/Y=");
  display1.print(mpu.GetAngY());
  display1.print("/Z=");
  display1.println(mpu.GetAngZ());
  display1.display();

  
  if (button1State == HIGH) {
//    digitalWrite(led1, HIGH);
    digitalWrite(led2, HIGH);
    digitalWrite(led3, HIGH);
    tone(buzzer, 440);
  }
  else {
//    digitalWrite(led1, LOW);
    digitalWrite(led2, LOW);
    digitalWrite(led3, LOW);
    noTone(buzzer);
  }
  delay(100);
  
}
