#include <Arduino.h>

// Define the motor pins on the Arduino
#define pinM1 5
#define pinM2 9
#define pinDir1M1 6
#define pinDir2M1 10
#define pinDir1M2 7
#define pinDir2M2 8

// Define LED pin
#define pinLED 2

void set_motor_speeds(signed int speed){
  /*
  * Method for writing speed to the motors. Sets the same amount of speed to the both motors.
  * Param: speed - Signed integer that corresponds to the wanted speed of the motor. Range of 0 - 255.
  */

  // The motors do no go lower than this.
  if(abs(speed) < 90){
    return;
  }

  // Set the direction of the motors
  if (speed > 0){
      digitalWrite(pinDir1M1, HIGH);
      digitalWrite(pinDir2M1, LOW);
      digitalWrite(pinDir1M2, LOW);
      digitalWrite(pinDir2M2, HIGH);
  }
  else{
      digitalWrite(pinDir1M1, LOW);
      digitalWrite(pinDir2M1, HIGH);
      digitalWrite(pinDir1M2, HIGH);
      digitalWrite(pinDir2M2, LOW);
  }
  // Write the speed of the motors
  analogWrite(pinM1, abs(speed));
  analogWrite(pinM2, abs(speed));
}

void setup() {
  // Start serial communication
  Serial.begin(9600);
  
  // Direction pins
  pinMode(pinDir1M1, OUTPUT);
  pinMode(pinDir2M1, OUTPUT);
  pinMode(pinDir1M2, OUTPUT);
  pinMode(pinDir2M2, OUTPUT);

  // Motor ctrl signal pins
  pinMode(pinM1, OUTPUT);
  pinMode(pinM2, OUTPUT);

}

void loop() {
  // Example run of the motors
  for (int i=100; i<=200; i=i+10){
    set_motor_speeds(i);
    delay(500);
  }
  for (int i=200; i>=100; i=i-10){
    set_motor_speeds(i);
    delay(500);
  }
  for (int i=-100; i>=-200; i=i-10){
    set_motor_speeds(i);
    delay(500);
  }
  for (int i=-200; i>=-100; i=i+10){
    set_motor_speeds(i);
    delay(500);
  }
}