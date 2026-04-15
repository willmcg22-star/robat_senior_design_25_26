// ============================================================
// Robat Multi Motor Control - Grand Central M4
// Version 2 - 14 Motor Serial Command Interface
//
// Command format (case-insensitive):
//   M<1-14> <f|b|s>
// Examples:
//   M1 f     -> Motor 1 forward
//   M9 b     -> Motor 9 backward
//   M3 s     -> Motor 3 stop
//   stop     -> Stop all motors immediately
//   enc      -> Print all encoder counts
//   enc M5   -> Print encoder count for Motor 5
//   reset    -> Reset all encoder counts to zero
//
// ENCODER STRATEGY:
//   Interrupt-driven: Motors 2, 3
//   Polled (all others): Motors 1,4,5,6,7,8,9,10,11,12,13,14
//   Known limitation: Motor 10 encoder unreliable due to
//   EXTINT6/7 hardware conflicts with Motors 2,9,11,13
//
// PIN ASSIGNMENTS:
//   Motor  | PH   | EN   | ENC1 | ENC2
//   -------|------|------|------|-----
//   M1     | D12  | D13  | D9   | D8
//   M2     | D10  | D11  | D7   | D6
//   M3     | D5   | D4   | D14  | D15
//   M4     | D3   | D2   | D16  | D17
//   M5     | D22  | D23  | D26  | D27
//   M6     | D24  | D25  | D28  | D29
//   M7     | D30  | D31  | D34  | D35
//   M8     | D32  | D33  | D36  | D37
//   M9     | D38  | D39  | D42  | D43
//   M10    | D40  | D41  | D44  | D45  (encoder unreliable - see above)
//   M11    | D50  | D51  | D46  | D47
//   M12    | D52  | D53  | D48  | D49
//   M13    | D70  | D69  | D74  | D73
//   M14    | D67  | D68  | D72  | D71
// ============================================================

#include <Arduino.h>

#define NUM_MOTORS 14

// ---- How long each motor runs when given f or b command ----
// Change this value to adjust run time (milliseconds)
// Examples: 500 = 0.5 sec, 1000 = 1 sec, 2000 = 2 sec
const int RUN_TIME_MS = 2000;

// ---- Motor struct ----
struct Motor {
  uint8_t phPin;
  uint8_t enPin;
  uint8_t enc1Pin;
  uint8_t enc2Pin;
  volatile long encoderCount;
  volatile uint8_t lastEncState;
};

// ---- All 14 motors ----
Motor motors[NUM_MOTORS] = {
  // PH,  EN,  ENC1, ENC2, count, lastState
  { 12,  13,  9,   8,   0, 0 },  // Motor 1  (polled)
  { 10,  11,  7,   6,   0, 0 },  // Motor 2  (interrupt)
  { 5,   4,   14,  15,  0, 0 },  // Motor 3  (interrupt)
  { 3,   2,   16,  17,  0, 0 },  // Motor 4  (polled)
  { 22,  23,  26,  27,  0, 0 },  // Motor 5  (polled)
  { 24,  25,  28,  29,  0, 0 },  // Motor 6  (polled)
  { 30,  31,  34,  35,  0, 0 },  // Motor 7  (polled)
  { 32,  33,  36,  37,  0, 0 },  // Motor 8  (polled)
  { 38,  39,  42,  43,  0, 0 },  // Motor 9  (polled)
  { 40,  41,  44,  45,  0, 0 },  // Motor 10 (polled - encoder unreliable)
  { 50,  51,  46,  47,  0, 0 },  // Motor 11 (polled)
  { 52,  53,  48,  49,  0, 0 },  // Motor 12 (polled)
  { 70,  69,  74,  73,  0, 0 },  // Motor 13 (polled)
  { 67,  68,  72,  71,  0, 0 },  // Motor 14 (polled)
};

// ---- Polled motor indices ----
const int POLLED_MOTORS[] = {0, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
const int NUM_POLLED = 12;

// ============================================================
// ENCODER QUADRATURE DECODE TABLE
// ============================================================
const int8_t QUAD_TABLE[16] = {
//  new: 00  01  10  11
     0,  +1, -1,  0,   // prev=00
    -1,   0,  0, +1,   // prev=01
    +1,   0,  0, -1,   // prev=10
     0,  -1, +1,  0    // prev=11
};

// ============================================================
// ENCODER HELPERS
// ============================================================
inline uint8_t readEncState(int idx) {
  return (digitalRead(motors[idx].enc1Pin) << 1) |
          digitalRead(motors[idx].enc2Pin);
}

inline void updateEncoder(int idx, uint8_t newState) {
  uint8_t oldState = motors[idx].lastEncState;
  motors[idx].encoderCount += QUAD_TABLE[(oldState << 2) | newState];
  motors[idx].lastEncState = newState;
}

void pollEncoders() {
  for (int i = 0; i < NUM_POLLED; i++) {
    int idx = POLLED_MOTORS[i];
    uint8_t newState = readEncState(idx);
    if (newState != motors[idx].lastEncState) {
      updateEncoder(idx, newState);
    }
  }
}

long getEncoderCount(int idx) {
  noInterrupts();
  long count = motors[idx].encoderCount;
  interrupts();
  return count;
}

void resetEncoderCount(int idx) {
  noInterrupts();
  motors[idx].encoderCount = 0;
  interrupts();
}

void resetAllEncoders() {
  for (int i = 0; i < NUM_MOTORS; i++) resetEncoderCount(i);
}

// ============================================================
// INTERRUPT HANDLERS - Motors 2 and 3 only
// ============================================================
void encISR_M2() { updateEncoder(1, readEncState(1)); }
void encISR_M3() { updateEncoder(2, readEncState(2)); }

// ============================================================
// MOTOR CONTROL
// ============================================================
void motorForward(int idx) {
  digitalWrite(motors[idx].phPin, HIGH);
  digitalWrite(motors[idx].enPin, HIGH);
}

void motorReverse(int idx) {
  digitalWrite(motors[idx].phPin, LOW);
  digitalWrite(motors[idx].enPin, HIGH);
}

void motorStop(int idx) {
  digitalWrite(motors[idx].phPin, LOW);
  digitalWrite(motors[idx].enPin, LOW);
}

void stopAllMotors() {
  for (int i = 0; i < NUM_MOTORS; i++) motorStop(i);
}

// ============================================================
// SERIAL COMMAND PARSING
// ============================================================

// Read one full line from Serial (until newline)
String readLine() {
  String line = Serial.readStringUntil('\n');
  line.trim();
  return line;
}

// Parse "M<1-14> <f|b|s>" -> returns true if valid
bool parseMotorCommand(const String& line, uint8_t &idxOut, char &dirOut) {
  if (line.length() < 3) return false;

  int sp = line.indexOf(' ');
  if (sp < 0) return false;

  String tokMotor = line.substring(0, sp);
  String tokDir   = line.substring(sp + 1);
  tokMotor.trim();
  tokDir.trim();
  if (tokMotor.length() < 2 || tokDir.length() < 1) return false;

  char mChar = tokMotor.charAt(0);
  if (mChar != 'M' && mChar != 'm') return false;

  int motorNum = tokMotor.substring(1).toInt();
  if (motorNum < 1 || motorNum > NUM_MOTORS) return false;

  char d = tolower(tokDir.charAt(0));
  if (d != 'f' && d != 'b' && d != 's') return false;

  idxOut = (uint8_t)(motorNum - 1);
  dirOut = d;
  return true;
}

// Parse "enc M<1-14>" -> returns true if valid, sets idxOut
bool parseEncCommand(const String& line, uint8_t &idxOut) {
  int sp = line.indexOf(' ');
  if (sp < 0) return false;

  String tokDir = line.substring(sp + 1);
  tokDir.trim();
  if (tokDir.length() < 2) return false;

  char mChar = tokDir.charAt(0);
  if (mChar != 'M' && mChar != 'm') return false;

  int motorNum = tokDir.substring(1).toInt();
  if (motorNum < 1 || motorNum > NUM_MOTORS) return false;

  idxOut = (uint8_t)(motorNum - 1);
  return true;
}

void printAllEncoders() {
  Serial.println("--- Encoder Counts ---");
  for (int i = 0; i < NUM_MOTORS; i++) {
    Serial.print("  M"); Serial.print(i + 1);
    Serial.print(": ");
    Serial.print(getEncoderCount(i));
    if (i == 9) Serial.print("  (unreliable - known hardware limit)");
    Serial.println();
  }
  Serial.println("----------------------");
}

void printHelp() {
  Serial.println("--- Commands ---");
  Serial.println("  M<1-14> f    -> Motor forward");
  Serial.println("  M<1-14> b    -> Motor backward");
  Serial.println("  M<1-14> s    -> Motor stop");
  Serial.println("  enc          -> Print all encoder counts");
  Serial.println("  enc M<1-14>  -> Print one motor encoder count");
  Serial.println("  reset        -> Reset all encoder counts");
  Serial.println("  stop         -> Stop all motors");
  Serial.println("  help         -> Show this menu");
  Serial.println("----------------");
}

// ============================================================
// SETUP
// ============================================================
void setup() {
  Serial.begin(115200);
  while (!Serial);
  delay(2000);
  Serial.println("Serial connected...");
  delay(1000);

  // Init all motor pins
  for (int i = 0; i < NUM_MOTORS; i++) {
    pinMode(motors[i].phPin,   OUTPUT);
    pinMode(motors[i].enPin,   OUTPUT);
    pinMode(motors[i].enc1Pin, INPUT_PULLUP);
    pinMode(motors[i].enc2Pin, INPUT_PULLUP);
    motorStop(i);
    motors[i].lastEncState = readEncState(i);
  }

  // Attach interrupts for Motors 2 and 3 only
  attachInterrupt(digitalPinToInterrupt(motors[1].enc1Pin), encISR_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[1].enc2Pin), encISR_M2, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].enc1Pin), encISR_M3, CHANGE);
  attachInterrupt(digitalPinToInterrupt(motors[2].enc2Pin), encISR_M3, CHANGE);

  Serial.println("Robat Motor Control Ready.");
  printHelp();
}

// ============================================================
// LOOP
// ============================================================
void loop() {
  // Always keep polling encoders for polled motors
  pollEncoders();

  // Only process serial input when available
  if (Serial.available() <= 0) return;

  String line = readLine();
  if (line.length() == 0) return;

  // ---- Global stop ----
  if (line.equalsIgnoreCase("stop") || line.equalsIgnoreCase("s")) {
    stopAllMotors();
    Serial.println("All motors stopped.");
    return;
  }

  // ---- Reset encoders ----
  if (line.equalsIgnoreCase("reset")) {
    resetAllEncoders();
    Serial.println("All encoder counts reset to zero.");
    return;
  }

  // ---- Print all encoders ----
  if (line.equalsIgnoreCase("enc")) {
    printAllEncoders();
    return;
  }

  // ---- Print single encoder ----
  if (line.substring(0, 3).equalsIgnoreCase("enc")) {
    uint8_t idx;
    if (parseEncCommand(line, idx)) {
      Serial.print("M"); Serial.print(idx + 1);
      Serial.print(" encoder: ");
      Serial.println(getEncoderCount(idx));
    } else {
      Serial.println("Bad enc command. Try: enc M5");
    }
    return;
  }

  // ---- Help ----
  if (line.equalsIgnoreCase("help") || line.equalsIgnoreCase("h")) {
    printHelp();
    return;
  }

  // ---- Motor command ----
  uint8_t idx;
  char dir;
  if (!parseMotorCommand(line, idx, dir)) {
    Serial.print("Unknown command: '");
    Serial.print(line);
    Serial.println("'. Type 'help' for options.");
    return;
  }

  // Execute motor command
  Serial.print("M"); Serial.print(idx + 1); Serial.print(" -> ");
  if (dir == 'f') {
    resetEncoderCount(idx);
    motorForward(idx);
    Serial.print("FORWARD for ");
    Serial.print(RUN_TIME_MS);
    Serial.println("ms...");
    unsigned long start = millis();
    while (millis() - start < RUN_TIME_MS) { pollEncoders(); }
    motorStop(idx);
    Serial.print("  Done. Encoder count: ");
    Serial.println(getEncoderCount(idx));
  } else if (dir == 'b') {
    resetEncoderCount(idx);
    motorReverse(idx);
    Serial.print("BACKWARD for ");
    Serial.print(RUN_TIME_MS);
    Serial.println("ms...");
    unsigned long start = millis();
    while (millis() - start < RUN_TIME_MS) { pollEncoders(); }
    motorStop(idx);
    Serial.print("  Done. Encoder count: ");
    Serial.println(getEncoderCount(idx));
  } else if (dir == 's') {
    motorStop(idx);
    Serial.println("STOP");
    Serial.print("  Encoder count: ");
    Serial.println(getEncoderCount(idx));
  }
}
