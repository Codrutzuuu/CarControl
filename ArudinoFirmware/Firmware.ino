#include <Arduino.h>

#define MOTOR_DIR_A 8
#define MOTOR_DIR_B 7
#define MOTOR_PWM_R 5
#define MOTOR_PWM_L 9
#define MOTOR_BRAKE 6
#define ENCODER_R 2
#define ENCODER_L 3

const unsigned long SERIAL_TIMEOUT = 1000;
const String COMMAND_END = "\r\n";
const int MAX_PWM = 40;
const int PWM_STEP = 2;
const unsigned long BRAKE_DELAY = 300;

volatile long encoderR = 0;
volatile long encoderL = 0;
int targetR = 0, targetL = 0;
int currentR = 0, currentL = 0;
unsigned long lastActivity = 0;

volatile unsigned long lastTimeR = 0;
volatile unsigned long lastTimeL = 0;

void handleEncoderR() {
  unsigned long now = micros();
  if (now - lastTimeR > 500) {
      if (digitalRead(MOTOR_DIR_A) == HIGH && digitalRead(MOTOR_DIR_B) == LOW) encoderR++;
    else if (digitalRead(MOTOR_DIR_A) == LOW && digitalRead(MOTOR_DIR_B) == HIGH) encoderR--;
    else if (digitalRead(MOTOR_DIR_A) == HIGH && digitalRead(MOTOR_DIR_B) == HIGH) encoderR--;
    else if (digitalRead(MOTOR_DIR_A) == LOW && digitalRead(MOTOR_DIR_B) == LOW) encoderR++; 
    lastTimeR = now;
  }
}

void handleEncoderL() {
  unsigned long now = micros();
  if (now - lastTimeL > 500) {
    
    if (digitalRead(MOTOR_DIR_A) == HIGH && digitalRead(MOTOR_DIR_B) == LOW) encoderL++;
    else if (digitalRead(MOTOR_DIR_A) == LOW && digitalRead(MOTOR_DIR_B) == HIGH) encoderL--;
    else if (digitalRead(MOTOR_DIR_A) == HIGH && digitalRead(MOTOR_DIR_B) == HIGH) encoderL++; 
    else if (digitalRead(MOTOR_DIR_A) == LOW && digitalRead(MOTOR_DIR_B) == LOW) encoderL--;  
    lastTimeL = now;
  }
}
void setMotorSpeed(int speedR, int speedL) {
  speedR = constrain(speedR, -MAX_PWM, MAX_PWM);
  speedL = constrain(speedL, -MAX_PWM, MAX_PWM);
  
  // Cazul 1: Fata (ambele motoare inainte)
  if (speedR > 0 && speedL > 0) {
    digitalWrite(MOTOR_DIR_A, HIGH);
    digitalWrite(MOTOR_DIR_B, LOW);
  }
  // Cazul 2: Spate (ambele motoare inapoi)
  else if (speedR < 0 && speedL < 0) {
    digitalWrite(MOTOR_DIR_A, LOW);
    digitalWrite(MOTOR_DIR_B, HIGH);
  }
  // Cazul 3: Dreapta (stanga inainte, dreapta inapoi)
  else if (speedR < 0 && speedL > 0) {
    digitalWrite(MOTOR_DIR_A, HIGH);
    digitalWrite(MOTOR_DIR_B, HIGH);
  }
  // Cazul 4: Stanga (dreapta inainte, stanga inapoi)
  else if (speedR > 0 && speedL < 0) {
    digitalWrite(MOTOR_DIR_A, LOW);
    digitalWrite(MOTOR_DIR_B, LOW);
  }

  analogWrite(MOTOR_PWM_R, abs(speedR));
  analogWrite(MOTOR_PWM_L, abs(speedL));
  digitalWrite(MOTOR_BRAKE, LOW);
  if (speedR == 0 && speedL == 0) {
    digitalWrite(MOTOR_BRAKE, HIGH);
  } else {
    digitalWrite(MOTOR_BRAKE, LOW);
    lastActivity = millis();
  }
}

void processCommand(String cmd) {
  cmd.trim();
  if (cmd.startsWith("m ")) {
    int space = cmd.indexOf(' ', 2);
    targetR = cmd.substring(2, space).toInt()*0.7;
    targetL = cmd.substring(space + 1).toInt()*0.7;
    Serial.print("ACK");
  } else if (cmd == "e") {
    Serial.print("e ");
    Serial.print(encoderR);
    Serial.print(" ");
    Serial.print(encoderL);
  } else if (cmd.startsWith("u ")) {
    Serial.print("PID_SET");
  } else {
    Serial.print("UNKNOWN_CMD");
  }
  Serial.print(COMMAND_END);
}

void setup() {
  pinMode(MOTOR_DIR_A, OUTPUT);
  pinMode(MOTOR_DIR_B, OUTPUT);
  pinMode(MOTOR_PWM_R, OUTPUT);
  pinMode(MOTOR_PWM_L, OUTPUT);
  pinMode(MOTOR_BRAKE, OUTPUT);
  digitalWrite(MOTOR_BRAKE, HIGH);
  pinMode(ENCODER_R, INPUT_PULLUP);
  pinMode(ENCODER_L, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_R), handleEncoderR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_L), handleEncoderL, CHANGE);
  Serial.begin(57600);
  while (!Serial);
  Serial.setTimeout(SERIAL_TIMEOUT);
  Serial.print("READY" + COMMAND_END);
}

void loop() {
  static unsigned long lastSend = 0;
  if (Serial.available()) {
    String input = Serial.readStringUntil('\r');
    processCommand(input);
  }
  if (currentR < targetR) currentR = min(currentR + PWM_STEP, targetR);
  else if (currentR > targetR) currentR = max(currentR - PWM_STEP, targetR);
  if (currentL < targetL) currentL = min(currentL + PWM_STEP, targetL);
  else if (currentL > targetL) currentL = max(currentL - PWM_STEP, targetL);
  setMotorSpeed(currentR, currentL);
  if (millis() - lastSend >= 50) {
    Serial.print("e ");
    Serial.print(encoderR);
    Serial.print(" ");
    Serial.print(encoderL);
    Serial.print(COMMAND_END);
    lastSend = millis();
  }
  if (millis() - lastActivity > BRAKE_DELAY) {
    digitalWrite(MOTOR_BRAKE, HIGH);
    targetR = targetL = currentR = currentL = 0;
  }
}