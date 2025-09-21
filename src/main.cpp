#include <Arduino.h>
#include <Servo.h>
#include <Stepper.h>

#define bluetooth Serial1

//SoftwareSerial bluetooth(13, 12); // 13 - TXD, 12 - RXD

const int IN1 = 22;
const int IN2 = 24;
const int ENA = 9;
const int IN3 = 26;
const int IN4 = 28;
const int ENB = 10;

const int stepsPerRevolution = 200;
Stepper baseStepper(stepsPerRevolution, 30, 32, 34, 36);

int stepperPos = 0;
int stepDirection = 0;
bool stepActive = false;

Servo baseServoR;
Servo baseServoL;
Servo gripperServo;
int angleR = 0;                     // Right side servo
int angleL = 180;                   // Left side servo
int angleG = 0;                     // Gripper servo

char carCommand;

void forward();
void reverse();
void leftTurn();
void rightTurn();
void stopMotors();
void executeMotion();
void runCar();
void moveServo();
void stepperMove();

void setup() {
  // put your setup code here, to run once:
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  baseServoR.attach(11);
  baseServoL.attach(12);
  //gripperServo.attach(44);
  baseStepper.setSpeed(60);

  baseServoR.write(angleR);
  baseServoL.write(angleL);
  gripperServo.write(angleG);

  Serial1.begin(9600);
  bluetooth.begin(9600);

  Serial.println("READY TO START!!!");

}

void loop() {
  // put your main code here, to run repeatedly:
  if (bluetooth.available()){
    carCommand = bluetooth.read();
    
        runCar();
        moveServo();
        stepperMove();
  }

  if (stepActive){
    baseStepper.step(stepDirection);
  } 

  if (carCommand != 'S') {  
    carCommand = '\0';
    }
}



void reverse(){
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, 200);  
  
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, 200); 
}

void forward(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, 200); 
}

void leftTurn(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 10);

  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENB, 200);  
}  

void rightTurn(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, 200);

  digitalWrite(IN4, HIGH);
  digitalWrite(IN3, LOW);
  analogWrite(ENB, 10);  
}  

void stopMotors(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(ENA, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  digitalWrite(ENB, LOW);  
}

void runCar(){
  if (carCommand == 'F'){
    forward();
    Serial.println("Moving Forward...");
  }
  else if (carCommand == 'B'){
    reverse();
    Serial.println("Reversing...");
  }
  else if (carCommand == 'L'){
    leftTurn();
    Serial.println("Turning left...");
  }
  else if (carCommand == 'R'){
    rightTurn();
    Serial.println("Turning right...");
  }
  else if (carCommand == 'S'){
    stopMotors();
    Serial.println("Car stopped");
  }
}

void moveServo(){
  if (carCommand == 'Y' && angleR < 60 && angleL > 120){
      angleR += 30;
      angleL -= 30;
      baseServoR.write(angleR);
      baseServoL.write(angleL);
      Serial.print("Servo Position: ");
      Serial.println(angleR);
      carCommand = 0;
    }
    else if (carCommand == 'Z' && angleR > 0 && angleL < 180){
      angleL += 30;
      angleR -= 30;
      baseServoR.write(angleR);
      baseServoL.write(angleL);
      Serial.print("Servo Position: ");
      Serial.println(angleR);
      carCommand = 0;
    }
    else if (carCommand == 'W' && angleG < 180){
      angleG += 20;
      gripperServo.write(angleG);
    }
    else if (carCommand == 'X' && angleG > 0){
      angleG -= 20;
      gripperServo.write(angleG);
    }
    
}

void stepperMove(){
  if (carCommand == 'U'){
    stepActive = true;
    stepDirection = 1;
    Serial.print("CLOCKWISE...");
    }

  if (carCommand == 'V'){
    stepActive = true;
    stepDirection = -1;
    Serial.print("COUNTER-CLOCKWISE...");
  }
  if (carCommand == 'v' || carCommand == 'u'){
    stepActive = false;
    Serial.print("Motor stopped");
  }
}