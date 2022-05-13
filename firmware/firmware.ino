#include <ArduinoJson.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

#define switch1 7
#define switch2 2
#define switch3 3
#define switch4 6
#define switch5 5
#define switch6 4

#define pSwitch1 9 //activate pot throttle
#define pSwitch2 8 // set limit

#define pot1 A2 //pot throttle
#define pot2 A1 //pot limit
#define stickThrottle A3 // 580 - 720;
#define Direction 11
#define MasterOn 10
#define Motor 12

#define voltMeter A7
#define ampMeter A6

#define vmpp 27


int throttle; // 0 - 1023
int currentDuty = 0; //0 - 100
int rawDuty = 0; // 0 - 178
int velocity; // 0 - 100
int aim;

double voltSense;
long ampSense;
double finalCurrent = 0.01;

double outputVoltage;
bool remote_override = false;
String byteRecieve = "5";

int counter;
int limit = 0;
int currentLimit = 13;


unsigned long startMillis;
unsigned long currentMillis;

bool overVolt = false;
bool underVolt = false;
bool isEnabled = false;

bool showDisplay = false;      //switch1
bool buttonPressed = false;    //switch2
bool changeDirection = false;  //switch3
bool solarMode = false;        //switch4
bool kill = false;             //switch5
bool testing = false;          //switch6

int switchTimer = 0;
int screenTimer = 0;
int screen = 4; // 0 voltage, 1 limit, 2 duty
int regulatedDuty = 180;


String inputString = "";
bool stringComplete = false;

int dataCounter = 0;


StaticJsonDocument<128> boatData;
LiquidCrystal_I2C lcd(0x27, 20, 4);
Servo outputServo;

void setup() {
  inputString.reserve(200);
  lcd.init();
  lcd.backlight();
  lcd.print("startup");
  startup();
  setPins();
  Serial.begin(9600);
  outputServo.attach(Motor);
  outputServo.write(0);
  delay(1000);
  while (true) {
    if (analogRead(stickThrottle) <= 475 && digitalRead(pSwitch1) == 1) {
      break;
    }
    //      Serial.println(digitalRead(pSwitch1));
    lcd.setCursor(0, 0);
    lcd.print("inactive");
    delay(100);

  }
}

void loop() {


  if (currentDuty == 0) {
    updateInputs();
  }
  voltSense = analogRead(voltMeter) / 25.1 ;
  ampSense = 0;
  for (int i = 0; i < 300; i++) {
    ampSense += analogRead(ampMeter);
  }
  finalCurrent = double(ampSense) / 40 ;

  digitalWrite(Direction, changeDirection);

  //Serial.println(ampSense);

  checkStatus();
  readThrottle();
  dataRecieve();
  lcdDisplay();




  if (solarMode != 1) {
    delay(20);
    if (velocity > currentDuty) {
      if (finalCurrent <= currentLimit) {
        currentDuty += 1;
        //delay(15);
      }
      if (finalCurrent >= (currentLimit + 1)) {

        currentDuty -= 1;
        delay(15);
      }

    }
    else if (velocity < currentDuty) {
      currentDuty -= 1;
    }

  }
  else {
    if (velocity >= 20) {
      if (voltSense < vmpp) {
        for (int i = 0; i < 3; i++) {
          voltSense = (float(analogRead(voltMeter)) / 25.1);
          if (voltSense >= vmpp) {
            break;
          }
          currentDuty -= 1;
          //Serial.println("duty decrease");
          rawDuty = map(currentDuty, 0, 100, 0, 180);
          rawDuty = setLimit(rawDuty, 0, 180);
          outputServo.write(rawDuty);
          delay(150);
        }
      }
      if (voltSense > vmpp) {

        for (int i = 0; i < 3; i++) {
          voltSense = (float(analogRead(voltMeter)) / 25.1);
          if (voltSense <= vmpp) {
            break;
          }
          currentDuty += 1;
          //Serial.println("duty increase");
          rawDuty = map(currentDuty, 0, 100, 0, 180);
          rawDuty = setLimit(rawDuty, 0, 180);
          outputServo.write(rawDuty);
          delay((currentDuty * 2) + 10);
        }
      }
    }
    else {
      currentDuty -= 1;
    }
  }




  currentDuty = setLimit(currentDuty, 0, 100);



  if (isEnabled == true) {

    if (currentDuty > 0) {
      digitalWrite(MasterOn, HIGH);
    }
    else {
      digitalWrite(MasterOn, LOW);
    }


  }
  else {
    screen = 4;
    digitalWrite(MasterOn, LOW);
    currentDuty = 0;
  }
  delay(30);
  //Serial.println(currentDuty);
  currentDuty = setLimit(currentDuty, 0 , limit);
  rawDuty = map(currentDuty, 0, 100, 0, 180);
  rawDuty = setLimit(rawDuty, 0, 180);
  outputServo.write(rawDuty);

  //writeData(velocity, currentDuty, voltSense * 100, solarMode, isEnabled, finalCurrent * 100); 

  dataCounter += 1;
  if (dataCounter >= 5){
    
    dataCounter = 0;
    //writeData(velocity, currentDuty, 24 * 100, voltSense * 100, underVolt, solarMode, isEnabled, finalCurrent * 100); // remove chip temp and undervolt
    // TP, DP, BV, solarMode, isEnabled, BC
    writeData(velocity, rawDuty, voltSense * 100, solarMode, isEnabled, finalCurrent * 100); 
  }
  //for(int i = 0; i< 100; i++){
  //  writeData(i,i,24,i, i, i, i, i);
  //  delay(100);
  //}
  //Serial.println(velocity);

  //Serial.println(finalCurrent);*/
}


//throttle percent 0 - 100, duty percent 0 - 180, chip temperature float, battery voltage double 0 - 40 ,
//undervolt boolean, and solar mode  boolean , enable boolean, battery current 0 - 30;

// TP, DP, BV, solarMode, isEnabled, BC
void writeData(int TP, int DP, int BV, int SM, int EN, int BC) {

  boatData["TP"] = TP;
  boatData["DP"] = DP;
  //boatData["CP"] = CP;
  boatData["BV"] = BV;
  //boatData["UV"] = UV;
  boatData["SM"] = SM;
  boatData["EN"] = EN;
  boatData["BC"] = BC;

  //Serial.println(BC);
  serializeMsgPack(boatData, Serial);
  Serial.print("\r\n");
}


void dataRecieve() {
  if (stringComplete) {
    byteRecieve = inputString;
    inputString = "";
    stringComplete = false;
  }
  if (byteRecieve == "5") {
    remote_override = false;
  }
  else {
    remote_override = true;
  }
  //  Serial.println(byteRecieve);
}


void startup() {
  underVolt = 0;
  overVolt = 0;
  solarMode = 0;
  isEnabled = 1;
  remote_override = 0;
}


void setPins() {
  pinMode(switch1, INPUT);
  pinMode(switch2, INPUT);
  pinMode(switch3, INPUT);
  pinMode(switch4, INPUT);
  pinMode(switch5, INPUT);
  pinMode(switch6, INPUT);
  pinMode(pSwitch1, INPUT);
  pinMode(pSwitch2, INPUT);
  pinMode(pot1, INPUT);
  pinMode(pot2, INPUT);
  pinMode(Direction, OUTPUT);
  pinMode(MasterOn, OUTPUT);
  pinMode(Motor, OUTPUT);
  pinMode(voltMeter, INPUT);
  pinMode(ampMeter, INPUT);
  pinMode(stickThrottle, INPUT);
}



void checkStatus() {
  if (digitalRead(switch5) == 0) {
    kill = true;
  }
  else {
    kill = false;
  }
  if (testing == false) {
    if (voltSense < 17.5) {
      underVolt = true;
    }
    else {
      underVolt = false;
    }
    if (voltSense > 25) {
      overVolt = true;
    }
    else {
      overVolt = false;
    }

    if (overVolt == false && underVolt == false && remote_override == false && kill == false) {
      isEnabled = true;
    }
    else {
      isEnabled = false;
    }


  }
  else {
    if (kill == true) {
      isEnabled = false;
    }
    else {
      isEnabled = true;
    }
  }
}

void updateInputs() {
  if (digitalRead(switch1) == 0) {
    showDisplay = true;
  }
  else {
    showDisplay = false;
  }

  if (digitalRead(switch3) == 0) {
    changeDirection = true;
  }
  else {
    changeDirection = false;
  }

  if (digitalRead(switch4) == 0) {
    solarMode = true;
  }
  else {
    solarMode = false;
  }



  if (digitalRead(switch6) == 0) {
    testing = true;
  }
  else {
    testing = false;
  }
  if (digitalRead(pSwitch2) == LOW) {
    outputVoltage = map(analogRead(pot2), 100, 900, 30, 20);
    limit = 100 * (outputVoltage / voltSense);
    limit = setLimit(limit, 0, 100);
  }
  else {
    outputVoltage = 31;
    limit = 100;
  }

  currentLimit = map(analogRead(pot1),50,1000,2,100);
  currentLimit = setLimit(currentLimit,2,20);

}

void readThrottle() {
  if (digitalRead(pSwitch1) == LOW) { //setVelocity
    velocity = map(analogRead(stickThrottle), 485, 725, 0, 100);
    velocity = setLimit(velocity, 0, 100);

  }
  else {
    velocity = 0;
  }
}

void lcdDisplay() {
  if (screenTimer <= 0) {
    screenTimer = 15;

    if (digitalRead(switch2) == 0) {
      screen += 1;
      delay(100);
    }
    if (screen > 6 || screen < 1) {
      screen = 1;
    }
    if (showDisplay == true) {
      lcd.clear();
      lcd.backlight();
      switch (screen) {

        case 1:
          lcd.setCursor(0, 0);
          lcd.print("(V) limit: ");
          lcd.print(outputVoltage);
          lcd.print("V");
          lcd.setCursor(0, 1);
          lcd.print("raw: ");
          lcd.print(limit);
          lcd.print("%");
          break;
        case 2:
          lcd.setCursor(0, 0);
          lcd.print("duty percent: ");
          lcd.setCursor(0, 1);
          lcd.print(int(rawDuty));
          break;
        case 3:
          lcd.setCursor(0, 0);
          lcd.print("RecV data: ");
          lcd.setCursor(0, 1);
          lcd.print(byteRecieve);
          break;
        case 4:
          lcd.setCursor(0, 0);
          lcd.print("Kill: ");
          lcd.print(kill);
          lcd.setCursor(10, 0);
          lcd.print("UVP: ");
          lcd.print(underVolt);
          lcd.setCursor(0, 1);
          lcd.print("Ovride: ");
          lcd.print(remote_override);
          lcd.setCursor(10, 1);
          lcd.print("OVP: ");
          lcd.print(overVolt);
          break;
        case 5:
          lcd.setCursor(0, 0);
          lcd.print("current: ");
          lcd.print(finalCurrent);
          lcd.print("A");
          lcd.setCursor(0, 1);
          lcd.print("Voltage: ");
          lcd.print(voltSense);
          lcd.print("V");
          break;
        case 6:
          lcd.setCursor(0,0);
          lcd.print("(C) limit:");
          lcd.print(currentLimit);
          lcd.print("A");
      }
    }
    else {
      lcd.noBacklight();
    }
  }
  else {
    screenTimer -= 1;
  }

}

int setLimit (int value, int lowerBound, int upperBound) {
  if (value < lowerBound) {
    value = lowerBound;
  }
  if (value > upperBound) {
    value = upperBound;
  }
  return value;
}


void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // add it to the inputString:
    // if the incoming character is a newline, set a flag so the main loop can
    // do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    else {
      inputString += inChar;

    }
  }
}

void measurePower() {
  

}
