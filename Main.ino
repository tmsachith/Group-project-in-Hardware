#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <NewPing.h>
#include <Servo.h>


LiquidCrystal_I2C lcd(0x27, 20, 4);


#define TRIGGER_PIN 15
#define ECHO_PIN 14
#define buzzerPin 47
#define servoPin 9
#define TRIGGER_PIN3 23
#define ECHO_PIN3 22
#define TRIGGER_PIN2 25
#define ECHO_PIN2 24
#define SPEED_SENSOR 8 

#define left 33
#define left1 32
#define left2 31
#define left3 30

#define right3 37
#define right2 36
#define right1 35
#define right 34



unsigned int pulses = 0;
float distancePerRevolution = 0.0816814;


const int leftMotorForwardPin = 6;
const int leftMotorBackwardPin = 7;
const int rightMotorForwardPin = 4;
const int rightMotorBackwardPin = 5;
const int leftMotorSpeedPin = 10;
const int rightMotorSpeedPin = 11;

const int mic1pin = A8;
const int mic2pin = A10;
const int mic3pin = A6;
const int mic4pin = A2;
const int mic5pin = A0;
const int mic6pin = A5;

const int ldrlistarr[] = {A9, A11, A7, A3, A1, A4};
int buttonpress = 20;

const int buttonPin1 = 18;
const int buttonPin2 =19;
const int buttonPin3 = 2;
const int buttonPin4 = 3;  
const int buttonPin5 = 52;  

volatile int buttonState1 = HIGH;
volatile int buttonState2 = HIGH;
volatile int buttonState3 = HIGH;
volatile int buttonState4 = HIGH;
volatile int buttonState5 = HIGH;
int currentPosition = 0;

////////////////////////////////////////////////////////////////////
int ldrarr[6];
int ldrMin;
const int sample_window = 3000;
int mic1, mic2, mic3, mic4, mic5, mic6, mic1_max, mic1_min, mic2_max, mic2_min, mic3_max, mic3_min, mic4_max, mic4_min, mic5_max, mic5_min, mic6_max, mic6_min, amp1, amp2, amp3, amp4, amp5, amp6;
unsigned long start_time, current_time;
int northdif, northeastdif, eastsouthdif, southdif, southwestdif, westnorthdif;
int valarr[6];
int maxValue;
Servo servoMotor;
NewPing sonar(TRIGGER_PIN, ECHO_PIN);
NewPing sonar2(TRIGGER_PIN2, ECHO_PIN2);
NewPing sonar3(TRIGGER_PIN3, ECHO_PIN3);

unsigned long currentTime;
enum Mode {
  IDLE,
  LIGHT_FOLLOW,
  SOUND_FOLLOW,
  LINE_FOLLOW
};

Mode currentMode = IDLE;
int receivedValue;

void setup() {
  Serial.begin(9600);
  pinMode(buzzerPin, OUTPUT);
  lcd.init();
  lcd.backlight();
  lcd.clear();
  playStartupSound();
  turnon();
  displayMenu();
  pinMode(leftMotorForwardPin, OUTPUT);
  pinMode(leftMotorBackwardPin, OUTPUT);
  pinMode(rightMotorForwardPin, OUTPUT);
  pinMode(rightMotorBackwardPin, OUTPUT);
  pinMode(leftMotorSpeedPin, OUTPUT);
  pinMode(rightMotorSpeedPin, OUTPUT);
  pinMode(buttonPin1, INPUT_PULLUP);
  pinMode(buttonPin2, INPUT_PULLUP);
  pinMode(buttonPin3, INPUT_PULLUP);
  pinMode(buttonPin4, INPUT_PULLUP);  
  pinMode(buttonPin5, INPUT_PULLUP);  
  pinMode(SPEED_SENSOR, INPUT);
  attachInterrupt(digitalPinToInterrupt(buttonPin1), buttonInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(buttonPin2), buttonInterrupt, FALLING); 
  attachInterrupt(digitalPinToInterrupt(buttonPin3), buttonInterrupt, FALLING); 
  attachInterrupt(digitalPinToInterrupt(buttonPin4), buttonInterrupt, FALLING);  
  // analogWrite(leftMotorSpeedPin, 255);
  // analogWrite(rightMotorSpeedPin, 255);
  servoMotor.attach(servoPin);

  pinMode(left,INPUT);
  pinMode(left1,INPUT);
  pinMode(left2,INPUT);
  pinMode(left3,INPUT);
  pinMode(right,INPUT);
  pinMode(right1,INPUT);
  pinMode(right2,INPUT);
  pinMode(right3,INPUT);

}
void loop() {

    if (Serial.available() > 0) {
    receivedValue = Serial.parseInt();
    Serial.print("Received Value: ");
    Serial.println(receivedValue);
  }
  buttonState5 = digitalRead(buttonPin5);

  if (buttonState5 == LOW || receivedValue ==40) {
    // Button is pressed, scroll the menu
    currentPosition = (currentPosition + 1) % 4;

    // Update the display based on the current position
    displayMenu();
    
    delay(100);  // Adjust delay as needed to debounce and control scrolling speed
  }


  if ((buttonpress == 0 || receivedValue ==10) && currentMode != LIGHT_FOLLOW) {
    lcd.clear();
    analogWrite(leftMotorSpeedPin, 255);
    analogWrite(rightMotorSpeedPin, 255);
    select();
    start();
    lcd.clear();
    currentMode = LIGHT_FOLLOW;
    lightfollow();

  } else if ((buttonpress == 1|| receivedValue ==20) && currentMode != SOUND_FOLLOW) {
    lcd.clear();
    analogWrite(leftMotorSpeedPin, 255);
    analogWrite(rightMotorSpeedPin, 255);
    select();
    start();
    lcd.clear();
    currentMode = SOUND_FOLLOW;
    soundFollow();

  } else if ((buttonpress == 2 || receivedValue ==30) && currentMode != LINE_FOLLOW) {
    lcd.clear();
    select();
    start();
    lcd.clear();
    currentMode = LINE_FOLLOW;
    lineFollow();
    
  } else if ((buttonpress == 3 || receivedValue ==50) && (currentMode == LIGHT_FOLLOW || currentMode == SOUND_FOLLOW || currentMode == LINE_FOLLOW)) {
    stopDriving(); 
    beepSequence();
    lcd.clear();
    currentMode = IDLE;
    displayMenu();
    
  }
  switch (currentMode) {
    case LIGHT_FOLLOW:
      delay(50); 
      lightfollow();
      break;
    case SOUND_FOLLOW: 
      soundFollow();
      break;
    case LINE_FOLLOW:
      lineFollow();
      break;
    case IDLE:
      displayMenu();
      break;
  }
}

void beepSequence() {
  for (int i = 0; i < 2; ++i) {
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
  }
}
void select() {
  for (int i = 0; i < 1; ++i) {
    digitalWrite(buzzerPin, HIGH);
    delay(100);
    digitalWrite(buzzerPin, LOW);
    delay(100);
  }
}


void lineFollow()
{
  // delay(300);

  if(digitalRead(left)==1 && digitalRead(right)==1 && digitalRead(left1)==1 && digitalRead(right1)==1 && digitalRead(left2)==1 && digitalRead(right2)==1 && digitalRead(left3)==1 && digitalRead(right3)==1){
    //Forward
    stopDriving1();

  }
  else if(digitalRead(left3)==1 && digitalRead(right3)==1){
    //Forward
   driveForward1();
   delay(5);

  }
  else if(digitalRead(left3)==1 && digitalRead(right3)==0){
    //Forward
   
   turnLeft1();
   delay(5);

  }
  else if(digitalRead(left3)==0 && digitalRead(right3)==1){
    //Forward
   turnRight1();
   delay(5);

  }
   else if(digitalRead(left3)==1 && digitalRead(left2)==1){
    //Forward
   turnLeft1();
   delay(5);

  }
  else if(digitalRead(right2)==1 && digitalRead(right3)==1){
    //Forward
   turnRight1();
   delay(5);

  }
  else if(digitalRead(left2)==1 && digitalRead(left1)==1){
    //Forward
   turnLeft1();
   delay(5);

  }
  else if(digitalRead(right1)==1 && digitalRead(right2)==1){
    //Forward
   turnRight1();
   delay(5);

  }
   else if(digitalRead(left1)==1 && digitalRead(left)==1){
    //Forward
   turnLeft1();
   delay(5);

  }
  else if(digitalRead(right)==1 && digitalRead(right1)==1){
    //Forward
   turnRight1();
   delay(5);

  }
  else if(digitalRead(left)==1 && digitalRead(right)==0 && digitalRead(left1)==0 && digitalRead(right1)==0 && digitalRead(left2)==0 && digitalRead(right2)==0 && digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    turnLeft1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==1 && digitalRead(left1)==0 && digitalRead(right1)==0 && digitalRead(left2)==0 && digitalRead(right2)==0 && digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    turnRight1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==0 && digitalRead(left1)==1 && digitalRead(right1)==0 && digitalRead(left2)==0 && digitalRead(right2)==0 && digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    turnLeft1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==0 && digitalRead(left1)==0 && digitalRead(right1)==1 && digitalRead(left2)==0 && digitalRead(right2)==0 && digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    turnRight1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==0 && digitalRead(left1)==0 && digitalRead(right1)==0 && digitalRead(left2)==1 && digitalRead(right2)==0 && digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    turnLeft1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==0 && digitalRead(left1)==0 && digitalRead(right1)==0 && digitalRead(left2)==0 && digitalRead(right2)==1 && digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    turnRight1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==0 && digitalRead(left1)==0 && digitalRead(right1)==0 && digitalRead(left2)==0 && digitalRead(right2)==0 && digitalRead(left3)==1 && digitalRead(right3)==0){
    //Forward
    turnLeft1();
    delay(5);

  }
  else if(digitalRead(left)==0 && digitalRead(right)==0 && digitalRead(left1)==0 && digitalRead(right1)==0 && digitalRead(left2)==0 && digitalRead(right2)==0 && digitalRead(left3)==0 && digitalRead(right3)==1){
    //Forward
    turnRight1();
    delay(5);

  }
  else if(digitalRead(left3)==0 && digitalRead(right3)==0){
    //Forward
    stopDriving1();

  }
  else{
    stopDriving1();
  }


}

void lightfollow() {
  delay(1000);
  int ldrValues[6];
  for (int i = 0; i < 6; i++) {
    ldrValues[i] = analogRead(ldrlistarr[i]);
  }
  int ldrMin1 = ldrValues[0];
  int minldrIndex1 = 0;
  int ldrMin2 = ldrValues[1];
  int minldrIndex2 = 1;
  if (ldrValues[0] > ldrValues[1]) {
    ldrMin1 = ldrValues[1];
    minldrIndex1 = 1;
    ldrMin2 = ldrValues[0];
    minldrIndex2 = 0;
  }
  for (int i = 2; i < 6; i++) {
    if (ldrValues[i] < ldrMin1) {
      ldrMin2 = ldrMin1;
      minldrIndex2 = minldrIndex1;
      ldrMin1 = ldrValues[i];
      minldrIndex1 = i;
    } else if (ldrValues[i] < ldrMin2) {
      ldrMin2 = ldrValues[i];
      minldrIndex2 = i;
    }
  }
  if (ldrMin1 < 500) {
    if ((ldrMin1 > 100) && ((ldrMin1 == ldrValues[0] && ldrMin2 == ldrValues[1]) || (ldrMin1 == ldrValues[1] && ldrMin2 == ldrValues[0]))) {
      handleForwardCase(ldrMin1);
    } else if ((ldrMin1 == ldrValues[3] && ldrMin2 == ldrValues[4]) || (ldrMin1 == ldrValues[4] && ldrMin2 == ldrValues[3])) {
      handleBackwardCase();
    } else {
      handleTurnCase(minldrIndex1);
    }
  }
}

void handleForwardCase(int ldrMin1) {
  rotateServo(90);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction: Forward");
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
      // lcd.clear();
      // lcd.setCursor(0, 0);
      // lcd.print("Speed: ");
      // lcd.print(getSpeed());
      // lcd.print(" cm/s");
      // delay(200); 
    driveForward();
    delay(800);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}

void handleForwardCaseS() {
  rotateServo(90);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction: Forward");
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
      // lcd.clear();
      // lcd.setCursor(0, 0);
      // lcd.print("Speed: ");
      // lcd.print(getSpeed());
      // lcd.print(" cm/s");
      // delay(200); 
    driveForward();
    delay(800);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}

void handleRightTurn(int delayTime, int servoAngle) {
  rotateServo(servoAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction:Right ");
  lcd.print(delayTime);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
    turnRight();
    delay(delayTime);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}

void handleRightTurn1(int delayTime, int servoAngle) {
  rotateServo(servoAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction:Right ");
  lcd.print(delayTime);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance2(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
    turnRight();
    delay(delayTime);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}

void handleLeftTurn(int delayTime, int servoAngle) {
  rotateServo(servoAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction:Left ");
  lcd.print(delayTime);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
    turnLeft();
    delay(delayTime);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}
void handleLeftTurn1(int delayTime, int servoAngle) {
  rotateServo(servoAngle);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction:Left ");
  lcd.print(delayTime);
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance3(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
    turnLeft();
    delay(delayTime);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}

void handleBackwardCase() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Direction: Backward");
  delay(100);
  lcd.setCursor(0, 1);
  lcd.print("Detecting Obstacles");
  delay(100);
  int currentDistance = distance2(); 
  delay(500);
  returnToInitialPosition();
  lcd.setCursor(0, 2);
  lcd.print("Distance: ");
  lcd.print(currentDistance);
  lcd.print(" cm");
  delay(200);
  if (currentDistance == 0 || currentDistance > 50) {
    lcd.setCursor(0, 3);
    lcd.print("Path Clear");
    delay(100);
    turnLeft();
    delay(1000);
    stopDriving();
  } else {
    buzzer();
    lcd.setCursor(0, 3);
    lcd.print("Path not clear");
    delay(100);
  }
}


void handleTurnCase(int minldrIndex1) {
  switch (minldrIndex1) {
    case 0:
      handleLeftTurn(250, 135);
      break;
    case 1:
      handleRightTurn(250, 45);
      break;
    case 2:
      handleRightTurn(500, 0);
      break;
    case 3:
      handleRightTurn1(750, 90);
      break;
    case 4:
      handleLeftTurn1(750, 90);
      break;
    case 5:
      handleLeftTurn(500, 180);
      break;
  }
}
void soundFollow() { 

  mic1_min = 1023;
  mic1_max = 0;
  mic2_min = 1023;
  mic2_max = 0;
  mic3_min = 1023;
  mic3_max = 0;
  mic4_min = 1023;
  mic4_max = 0;
  mic5_min = 1023;
  mic5_max = 0;
  mic6_min = 1023;
  mic6_max = 0;

  start_time = millis();    
  current_time = millis();
  
  while( millis() - start_time < sample_window){

    mic1 = analogRead(mic1pin);
    mic2 = analogRead(mic2pin);
    mic3 = analogRead(mic3pin);
    mic4 = analogRead(mic4pin);
    mic5 = analogRead(mic5pin);
    mic6 = analogRead(mic6pin);

    mic1_min = min(mic1, mic1_min);
    mic1_max = max(mic1, mic1_max);
    mic2_min = min(mic2, mic2_min);
    mic2_max = max(mic2, mic2_max);
    mic3_min = min(mic3, mic3_min);
    mic3_max = max(mic3, mic3_max);
    mic4_min = min(mic4, mic4_min);
    mic4_max = max(mic4, mic4_max);
    mic5_min = min(mic5, mic5_min);
    mic5_max = max(mic5, mic5_max);
    mic6_min = min(mic6, mic6_min);
    mic6_max = max(mic6, mic6_max);

    current_time = millis(); 
  }

  amp1 = mic1_max - mic1_min;
  amp2 = mic2_max - mic2_min;
  amp3 = mic3_max - mic3_min;
  amp4 = mic4_max - mic4_min;
  amp5 = mic5_max - mic5_min;
  amp6 = mic6_max - mic6_min;

  northdif = amp1 - amp2;
  northeastdif = amp2 - amp3;
  eastsouthdif = amp3 - amp4;
  southdif = amp4 - amp5;
  southwestdif = amp5 - amp6;
  westnorthdif = amp6 - amp1;

  valarr[0] = amp1 + amp2;
  valarr[1] = amp2 + amp3;
  valarr[2] = amp3 + amp4;
  valarr[3] = amp4 + amp5;
  valarr[4] = amp5 + amp6;
  valarr[5] = amp6 + amp1;


   maxValue = valarr[0];
  int maxIndex = 0;

  for (int i = 1; i < 6; i++) {
  if (valarr[i] > maxValue) {
    maxValue = valarr[i];
    maxIndex = i;
  }
}

  switch(maxIndex)
  {
    case 0:
    {
      if(northdif > 10)
      {
        handleLeftTurn(250, 135);
      }
      else if(northdif < -20)
      {
        handleRightTurn(250, 45);
      }
      else if(northdif < 10 && northdif > -20)
      {
        handleForwardCaseS();
      }
      break;
    }
    case 1:
    {
      if(northeastdif > 20)
      {
        handleRightTurn(250, 45);
      }
      else if(northeastdif < -50)
      {
        handleRightTurn(500, 0);
      }

      break;
    }
    case 2:
    {
      if(eastsouthdif > 20)
      {
        handleRightTurn(500, 0);
      }
      else if(eastsouthdif < -20)
      {
        handleRightTurn1(750, 0);
      }

      break;
    }
    case 3:
    {
      if(southdif > 20)
      {
        handleRightTurn1(750, 0);
      }
      else if(southdif < -20)
      {
        handleLeftTurn1(750, 180);
      }
      else if(southdif < 20 && southdif > -20)
      {
        handleBackwardCase();
      }

      break;
    }
    case 4:
    {
      if(southwestdif > 10)
      {
        handleLeftTurn1(750, 180);
      }
      else if(southwestdif < -100)
      {
          handleLeftTurn(500, 180);
      }

      break;
    }
    case 5:
    {
      if(westnorthdif > 100)
      {
        handleLeftTurn(500, 180);
      }
      else if(westnorthdif < -10)
      {
        handleLeftTurn(250, 135);
      }

      break;
    }
  }
}


void driveForward() {
  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void turnRight() {
  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
}

void turnLeft() {
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void stopDriving() {
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
}

void driveForward1() {
  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  analogWrite(leftMotorSpeedPin, 200);

  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
  analogWrite(rightMotorSpeedPin, 200);
}

void turnLeft1() {
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, HIGH);
  analogWrite(leftMotorSpeedPin, 215);

  digitalWrite(rightMotorForwardPin, HIGH);
  digitalWrite(rightMotorBackwardPin, LOW);
  analogWrite(rightMotorSpeedPin, 215);
}

void turnRight1() {
  digitalWrite(leftMotorForwardPin, HIGH);
  digitalWrite(leftMotorBackwardPin, LOW);
  analogWrite(leftMotorSpeedPin, 215);

  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, HIGH);
  analogWrite(rightMotorSpeedPin, 215);
}

void stopDriving1() {
  digitalWrite(leftMotorForwardPin, LOW);
  digitalWrite(leftMotorBackwardPin, LOW);
  analogWrite(leftMotorSpeedPin, 0);

  digitalWrite(rightMotorForwardPin, LOW);
  digitalWrite(rightMotorBackwardPin, LOW);
  analogWrite(rightMotorSpeedPin, 0);
}

void rotateServo(int angle) {
  servoMotor.write(angle);
  delay(50);
}

void buzzer(){
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);
  delay(250);
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);
  delay(250);
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);
  delay(250);
  digitalWrite(buzzerPin, HIGH);
  delay(250);
  digitalWrite(buzzerPin, LOW);
}

int distance() {
  currentTime = millis();
  static unsigned long lastUpdateTime = 0;
  int distance = sonar.ping_cm();
  delay(500);
  lastUpdateTime = currentTime;
  return distance;
}

int distance2() {
  currentTime = millis();
  static unsigned long lastUpdateTime = 0;
  int distance2 = sonar2.ping_cm();
  delay(500);
  lastUpdateTime = currentTime;
  return distance2;
}

int distance3() {
  currentTime = millis();
  static unsigned long lastUpdateTime = 0;
  int distance3 = sonar3.ping_cm();
  delay(500);
  lastUpdateTime = currentTime;
  return distance3;
}

void turnon() {
  lcd.setCursor(0, 1);
  lcd.print(" SYSTEM TURNING ON");
  delay(100);
 
  lcd.setCursor(0, 2);
  for (int i = 0; i < 20; ++i) {
    lcd.print(".");
    delay(100);
  }
  delay(200);
  lcd.clear();
}
void start() {
  lcd.setCursor(0, 1);
  lcd.print(" STARTING FUNCTION ");
  delay(100);
 
  lcd.setCursor(0, 2);
  for (int i = 0; i < 20; ++i) {
    lcd.print(".");
    delay(100);
  }
  delay(200);
  lcd.clear();
}

void playStartupSound(){

 
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(150);


  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(150);

  
  digitalWrite(buzzerPin, HIGH);
  delay(100);
  digitalWrite(buzzerPin, LOW);
  delay(150);


  delay(100);
  digitalWrite(buzzerPin, LOW);  
 }

void displayMenu() {
  lcd.setCursor(4, 0);
  lcd.print("- Main Menu -");

  // Display the menu items based on the current position
  switch (currentPosition) {
    case 0:
      lcd.setCursor(0, 2);
      lcd.print("White :-Sound Follow");
      lcd.setCursor(0, 3);
      lcd.print("Blue  :-Light Follow");
      break;

    case 1:
      lcd.setCursor(0, 2);
      lcd.print("Blue  :-Light Follow");
      lcd.setCursor(0, 3);
      lcd.print("Green :-Line Follow");
      break;

    case 2:
      lcd.setCursor(0, 2);
      lcd.print("Green :-Line Follow");
      lcd.setCursor(0, 3);
      lcd.print("Red   :-Back to Menu");
      break;

    case 3:
      lcd.setCursor(0, 2);
      lcd.print("Red   :-Return Home");
      lcd.setCursor(0, 3);
      lcd.print("White :-Sound Follow");
      break;
  }
}

void returnToInitialPosition() {
  servoMotor.write(90);
  delay(500);  
}

float getSpeed() {
  unsigned long startTime = millis();
  unsigned int pulseCopy = 10;

  while (millis() - startTime < 1000) {
    if (digitalRead(SPEED_SENSOR) == HIGH) {
      pulseCopy++;
      while (digitalRead(SPEED_SENSOR) == HIGH) {} 
    }
  }
  float speed = (pulseCopy / 20.0) * (distancePerRevolution * 10); 
  return speed;
}

void buttonInterrupt() {

  if(digitalRead(buttonPin1)==0 )
  {
    buttonpress = 0;
  }
  else if(digitalRead(buttonPin2)==0)
  {
    buttonpress = 1;
  }
  else if(digitalRead(buttonPin3)==0)
  {
    buttonpress = 2;
  }
  else if(digitalRead(buttonPin4)==0)
  {
    buttonpress = 3;
  }
}