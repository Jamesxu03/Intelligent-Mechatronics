/*
 * rps_firmware.ino - Rock-Paper-Scissors Robotic Hand Firmware
 *
 * Hardware: Custom PCB with 5x servo fingers, arm servo, 2x flex sensors,
 *           3x countdown LEDs, 3x result LEDs (Win/Lose/Tie), button.
 * Communication: Serial @ 115200 baud, transmits PHYSICAL_MOVE:<code> to host PC.
 */

#include <Arduino.h>

// ---------- Pin Assignments ----------
const int servoFinger0 = 3;   // Index finger servo
const int servoFinger1 = 5;   // Middle finger servo
const int servoFinger2 = 6;   // Ring finger servo
const int servoFinger3 = 9;   // Little finger servo
const int servoFinger4 = 10;  // Thumb servo
const int servoArm     = 11;  // Arm rotation servo

const int flexA = A0;         // Flex sensor A (index/middle)
const int flexB = A1;         // Flex sensor B (ring/little)
const int buttonPin = 2;      // Start button (INPUT_PULLUP)

const int ledCount1 = 4;      // Countdown LED 1
const int ledCount2 = 7;      // Countdown LED 2
const int ledCount3 = 8;      // Countdown LED 3
const int ledWin    = A2;     // Result LED: human wins
const int ledLose   = A3;     // Result LED: robot wins
const int ledTie    = A4;     // Result LED: draw

// ---------- Servo Constants ----------
const int FINGER_OPEN  = 180;
const int FINGER_CLOSE = 0;
const int ARM_CENTER   = 90;
const int ARM_LEFT90   = 0;
const int ARM_RIGHT90  = 180;

// ---------- Timing Constants (ms) ----------
const int t_count_step        = 400;
const int t_arm_step           = 600;
const int t_wait_before_start  = 500;
const int t_after_gesture      = 1000;
const int t_result_show        = 3000;

// ---------- PWM Helper ----------
const float minDuty = 2.5;   // 0 degrees
const float maxDuty = 12.5;  // 180 degrees

// Write an angle (0..180) to a servo pin using map/pwmWrite
void writeServoAngle(int pin, int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  float duty = map(angle, 0, 180, (int)(minDuty * 1000), (int)(maxDuty * 1000)) / 1000.0;
  pwmWrite(pin, duty, 50);
}

// Set robot hand pose by move code: 0=ROCK, 1=PAPER, 2=SCISSORS
void setRobotPose(int move) {
  if (move == 0) { // ROCK: all fingers closed
    writeServoAngle(servoFinger0, FINGER_CLOSE);
    writeServoAngle(servoFinger1, FINGER_CLOSE);
    writeServoAngle(servoFinger2, FINGER_CLOSE);
    writeServoAngle(servoFinger3, FINGER_CLOSE);
    writeServoAngle(servoFinger4, FINGER_CLOSE);
  } else if (move == 1) { // PAPER: all open
    writeServoAngle(servoFinger0, FINGER_OPEN);
    writeServoAngle(servoFinger1, FINGER_OPEN);
    writeServoAngle(servoFinger2, FINGER_OPEN);
    writeServoAngle(servoFinger3, FINGER_OPEN);
    writeServoAngle(servoFinger4, FINGER_OPEN);
  } else { // SCISSORS: index+middle open, others closed
    writeServoAngle(servoFinger0, FINGER_OPEN);   // index
    writeServoAngle(servoFinger1, FINGER_OPEN);   // middle
    writeServoAngle(servoFinger2, FINGER_CLOSE);  // ring
    writeServoAngle(servoFinger3, FINGER_CLOSE);  // little
    writeServoAngle(servoFinger4, FINGER_CLOSE);  // thumb
  }
}

// Read human move from two flex sensors: returns 0=ROCK,1=PAPER,2=SCISSORS
int readHumanMove(float baselineA, float baselineB) {
  const int samples = 20;
  float sumA = 0.0;
  float sumB = 0.0;
  for (int i = 0; i < samples; ++i) {
    sumA += analogRead(flexA);
    sumB += analogRead(flexB);
    delay(5);
  }
  float vA = sumA / samples;
  float vB = sumB / samples;
  
  const float THRESH = 0.05; 
  bool bentA = (vA > baselineA + THRESH);
  bool bentB = (vB > baselineB + THRESH);
  if (bentA && bentB) return 0;        // ROCK
  if (!bentA && !bentB) return 1;      // PAPER
  return 2;                            // SCISSORS (one bent)
}

// Simple decide winner: 0 draw, 1 human wins, -1 robot wins
int decideWinner(int human, int robot) {
  if (human == robot) return 0;
  if ((human == 0 && robot == 2) ||
      (human == 1 && robot == 0) ||
      (human == 2 && robot == 1)) return 1;
  return -1;
}

// Show result with result LEDs
void showResultLED(int result) {
  digitalWrite(ledWin, LOW);
  digitalWrite(ledLose, LOW);
  digitalWrite(ledTie, LOW);
  if (result == 0) { // draw
    digitalWrite(ledTie, HIGH);
    Serial.println("Result: DRAW");
  } else if (result == 1) { // human wins
    digitalWrite(ledWin, HIGH);
    Serial.println("Result: HUMAN WINS");
  } else { // robot wins
    digitalWrite(ledLose, HIGH);
    Serial.println("Result: ROBOT WINS");
  }
}

// Countdown rhythm using three leds
void countdownRhythm() {
  digitalWrite(ledCount1, HIGH);
  digitalWrite(ledCount2, HIGH);
  digitalWrite(ledCount3, HIGH);
  delay(t_count_step);
  digitalWrite(ledCount3, LOW);
  delay(t_count_step);
  digitalWrite(ledCount2, LOW);
  delay(t_count_step);
  digitalWrite(ledCount1, LOW);
  delay(t_count_step);
}

void armWiggleAndShowGesture(int robotMove) {
  writeServoAngle(servoArm, ARM_CENTER);
  delay(150);
  for (int a = ARM_CENTER; a <= ARM_RIGHT90; a += 5) {
    writeServoAngle(servoArm, a);
    delay(t_arm_step / 3);
  }
  for (int a = ARM_RIGHT90; a >= ARM_CENTER; a -= 5) {
    writeServoAngle(servoArm, a);
    delay(t_arm_step / 3);
  }
  for (int a = ARM_CENTER; a >= ARM_LEFT90; a -= 5) {
    writeServoAngle(servoArm, a);
    delay(t_arm_step / 3);
  }
  for (int a = ARM_LEFT90; a <= ARM_CENTER; a += 5) {
    writeServoAngle(servoArm, a);
    setRobotPose(robotMove); // reveal happens here
    delay(t_arm_step / 3);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("RPS Game starting...");
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledCount1, OUTPUT); pinMode(ledCount2, OUTPUT); pinMode(ledCount3, OUTPUT);
  pinMode(ledWin, OUTPUT); pinMode(ledLose, OUTPUT); pinMode(ledTie, OUTPUT);
  pinMode(servoFinger0, OUTPUT); pinMode(servoFinger1, OUTPUT);
  pinMode(servoFinger2, OUTPUT); pinMode(servoFinger3, OUTPUT);
  pinMode(servoFinger4, OUTPUT); pinMode(servoArm, OUTPUT);
  
  setRobotPose(1);
  writeServoAngle(servoArm, ARM_CENTER);
  
  float rv = analogRead(2);
  randomSeed((unsigned long)(rv * 100000.0f));
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    delay(30);
    if (digitalRead(buttonPin) != LOW) return; 
    while (digitalRead(buttonPin) == LOW) delay(5);
    
    Serial.println("Button pressed - starting round");
    delay(t_wait_before_start); 
    countdownRhythm();
    
    int robotMove = random(0, 3);
    armWiggleAndShowGesture(robotMove);
    delay(t_after_gesture);
    
    Serial.println("Calibrating flex sensors...");
    float sumA = 0, sumB = 0;
    const int calSamples = 30;
    for (int i = 0; i < calSamples; ++i) {
      sumA += analogRead(flexA);
      sumB += analogRead(flexB);
      delay(30);
    }
    float baseA = sumA / calSamples;
    float baseB = sumB / calSamples;
    delay(300); 
    
    int humanMove = readHumanMove(baseA, baseB);
    int result = decideWinner(humanMove, robotMove);
    showResultLED(result);
    
    // Relay result locally over serial to PC software for visual redundancy.
    Serial.print("PHYSICAL_MOVE:");
    Serial.println(humanMove);
    
    delay(t_result_show);
    digitalWrite(ledWin, LOW); digitalWrite(ledLose, LOW); digitalWrite(ledTie, LOW);
    setRobotPose(1);
    writeServoAngle(servoArm, ARM_CENTER);
  }
  delay(50);
}
