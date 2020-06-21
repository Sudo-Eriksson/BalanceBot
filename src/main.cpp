#include <Arduino.h>

// Define the motor pins on the Arduino
#define pinM1 5
#define pinM2 9
#define pinDir1M1 6
#define pinDir2M1 10
#define pinDir1M2 7
#define pinDir2M2 8

void setup() {
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
  // Set directions of the motors
  digitalWrite(pinDir1M1, HIGH);
  digitalWrite(pinDir2M1, LOW);
  digitalWrite(pinDir1M2, LOW);
  digitalWrite(pinDir2M2, HIGH);

  digitalWrite(pinM1, 80);
  digitalWrite(pinM2, 80);
}