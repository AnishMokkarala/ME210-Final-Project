#include <Arduino.h>

const int motor1pin1 = PIN_A0;
const int motor1pin2 = PIN_A1;
const int motor2pin1 = PIN_A4;
const int motor2pin2 = PIN_A5;

void setup() {
  // Pin Modes
  pinMode(motor1pin1, OUTPUT);
  pinMode(motor1pin2, OUTPUT);
  pinMode(motor2pin1, OUTPUT);
  pinMode(motor2pin2, OUTPUT);
  Serial.begin(9600);

//  digitalWrite(motor1pin1, HIGH);
//  digitalWrite(motor1pin2, LOW);
//  digitalWrite(motor2pin1, HIGH);
//  digitalWrite(motor2pin2, LOW);
}

void loop() {
  digitalWrite(motor1pin1, HIGH);
  digitalWrite(motor1pin2, LOW);
  digitalWrite(motor2pin1, HIGH);
  digitalWrite(motor2pin2, LOW);

}
