/*
  Smart RC Car - Arduino Uno + HotRC CT6A

  CH1 (D8): steering      CH2 (D9): throttle      CH3 (D10): mode switch
  L298N: ENA=5 IN1=6 IN2=7 ENB=3 IN3=2 IN4=4
  TCRT5000: left=11 centre=12 right=13 (digital outputs)
  LCD I2C: A4/A5          servo: A3               SRF05: trig=A0 echo=A1
*/

#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Servo.h>

// ---- Pin assignment --------------------------------------------------------
const byte ENA = 5, IN1 = 6, IN2 = 7;
const byte ENB = 3, IN3 = 4, IN4 = 2;
const byte CH1_PIN = 8, CH2_PIN = 9, CH3_PIN = 10;
const byte TCRT_LEFT = 11, TCRT_CENTRE = 12, TCRT_RIGHT = 13;
const byte SERVO_PIN = A3, TRIG_PIN = A0, ECHO_PIN = A1;

LiquidCrystal_I2C lcd(0x27, 16, 2); // Change 0x27 to 0x3F if your LCD is blank
Servo scanner;

// ---- Adjust these after testing -------------------------------------------
const bool CLIFF_IS_LOW = true; // TCRT output when sensor sees NO floor.
const int OBSTACLE_CM = 5;      // stop when the front object is 5 cm away
const int BACKUP_CLEAR_CM = 10; // reverse until front clearance is at least this
const unsigned long MAX_BACKUP_MS = 1600;
const unsigned long MANUAL_SERVO_SETTLE_MS = 180;
const int FOLLOW_MIN_CM = 14;   // stop this close to the tracked object
const int FOLLOW_MAX_CM = 90;   // ignore objects farther than this
const int AUTO_SPEED = 160;
const int TURN_SPEED = 170;
const int RC_DEAD_BAND = 45;
// L298N loses voltage, so many DC motors will not start at low PWM.
// Raise this gradually (e.g. 110, 125, 140) until both wheels start reliably.
const int MANUAL_START_PWM = 125;

enum Mode { MANUAL, AVOID, FOLLOW };
Mode previousMode = MANUAL;

// Manual-mode scanner state.  The two-phase update gives the servo time to
// reach its new angle before the SRF05 measurement is taken.
byte manualScanAngle = 35;
int manualScanStep = 1;
bool manualServoSettling = false;
unsigned long manualServoMovedAt = 0;
bool manualObstacleLocked = false;
byte manualClearReads = 0;
int manualDistanceCm = 400;

void setMotor(byte en, byte a, byte b, int speedValue) {
  speedValue = constrain(speedValue, -255, 255);
  if (speedValue > 0) {
    digitalWrite(a, HIGH); digitalWrite(b, LOW);
  } else if (speedValue < 0) {
    digitalWrite(a, LOW);  digitalWrite(b, HIGH);
  } else {
    digitalWrite(a, LOW);  digitalWrite(b, LOW); // brake/coast stop
  }
  analogWrite(en, abs(speedValue));
}

void drive(int leftSpeed, int rightSpeed) {
  setMotor(ENA, IN1, IN2, leftSpeed);
  setMotor(ENB, IN3, IN4, rightSpeed);
}

void stopCar() { drive(0, 0); }
void forward(int speedValue) { drive(speedValue, speedValue); }
void backward(int speedValue) { drive(-speedValue, -speedValue); }
void turnLeft(int speedValue) { drive(-speedValue, speedValue); }
void turnRight(int speedValue) { drive(speedValue, -speedValue); }

int readChannel(byte pin) {
  // A radio receiver normally sends 1000..2000 us pulses every 20 ms.
  unsigned long us = pulseIn(pin, HIGH, 25000);
  return (us >= 900 && us <= 2100) ? us : 1500; // signal-loss = neutral
}

Mode readMode() {
  int ch3 = readChannel(CH3_PIN);
  // CT6A three-position switch: low = auto, middle = manual, high = follow.
  // Swap AVOID and FOLLOW below if the physical switch direction is reversed.
  if (ch3 < 1300) return AVOID;
  if (ch3 > 1700) return FOLLOW;
  return MANUAL;
}

bool rawCliff(byte pin) {
  int value = digitalRead(pin);
  return CLIFF_IS_LOW ? value == LOW : value == HIGH;
}

bool cliffAhead() {
  return rawCliff(TCRT_LEFT) || rawCliff(TCRT_CENTRE) || rawCliff(TCRT_RIGHT);
}

int readDistanceCm() {
  digitalWrite(TRIG_PIN, LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 400; // no echo: regard as clear
  return constrain((int)(duration / 58UL), 2, 400);
}

int lookAt(byte angle) {
  scanner.write(angle);
  delay(230);                    // allow servo to settle before measuring
  int cm = readDistanceCm();
  delay(15);
  return cm;
}

void showMode(Mode mode) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("smart rc car");
  lcd.setCursor(0, 1);
  if (mode == MANUAL) lcd.print("manual");
  else if (mode == AVOID) lcd.print("auto avoid");
  else lcd.print("follow object");
}

void showManualDistance() {
  lcd.setCursor(0, 0);
  lcd.print("manual D:");
  if (manualDistanceCm < 100) lcd.print(' ');
  if (manualDistanceCm < 10) lcd.print(' ');
  lcd.print(manualDistanceCm);
  lcd.print("cm  ");
  lcd.setCursor(0, 1);
  if (manualObstacleLocked) lcd.print("OBST: ONLY BACK ");
  else lcd.print("scan left/right ");
}

void updateManualObstacleMonitor() {
  if (!manualServoSettling) {
    scanner.write(manualScanAngle);
    manualServoMovedAt = millis();
    manualServoSettling = true;
    return;
  }
  if (millis() - manualServoMovedAt < MANUAL_SERVO_SETTLE_MS) return;

  manualDistanceCm = readDistanceCm();
  if (manualDistanceCm <= OBSTACLE_CM) {
    // Lock forward motion. It is only cleared after a complete sweep finds
    // no close obstacle, so the next side-looking sample cannot unlock it.
    manualObstacleLocked = true;
    manualClearReads = 0;
  } else if (manualObstacleLocked) {
    manualClearReads++;
    if (manualClearReads >= 4) manualObstacleLocked = false;
  }
  showManualDistance();

  if (manualScanAngle == 145) manualScanStep = -1;
  else if (manualScanAngle == 35) manualScanStep = 1;
  manualScanAngle += manualScanStep * 55; // 35, 90, 145 degrees
  manualServoSettling = false;
}

void manualControl() {
  updateManualObstacleMonitor();
  int throttle = readChannel(CH2_PIN) - 1500;
  int steering = readChannel(CH1_PIN) - 1500;
  if (abs(throttle) < RC_DEAD_BAND) throttle = 0;
  if (abs(steering) < RC_DEAD_BAND) steering = 0;

  if (manualObstacleLocked) {
    // An obstacle at <= 5 cm was found.  Do not allow a turn-in-place or
    // forward motor command; pulling CH2 backward is the only allowed motion.
    if (throttle < 0) {
      int reverseSpeed = map(abs(throttle), 1, 500, MANUAL_START_PWM, 255);
      backward(reverseSpeed);
    } else {
      stopCar();
    }
    return;
  }

  // Mix CH1 and CH2: steering also works while moving forward/backward.
  int base = map(throttle, -500, 500, -255, 255);
  int steer = map(steering, -500, 500, -180, 180);
  int leftSpeed = constrain(base + steer, -255, 255);
  int rightSpeed = constrain(base - steer, -255, 255);

  // Convert every non-zero manual command into usable motor PWM.  Without
  // this, small stick movements may generate PWM too weak to overcome the
  // L298N's voltage drop and the motor's static friction.
  if (leftSpeed != 0) {
    leftSpeed = (leftSpeed > 0 ? 1 : -1) *
                map(abs(leftSpeed), 1, 255, MANUAL_START_PWM, 255);
  }
  if (rightSpeed != 0) {
    rightSpeed = (rightSpeed > 0 ? 1 : -1) *
                 map(abs(rightSpeed), 1, 255, MANUAL_START_PWM, 255);
  }
  drive(leftSpeed, rightSpeed);
}

void escapeCliff() {
  stopCar(); delay(100);
  backward(AUTO_SPEED); delay(500);
  // Prefer the side whose TCRT sensor still sees floor.
  bool leftDanger = rawCliff(TCRT_LEFT);
  bool rightDanger = rawCliff(TCRT_RIGHT);
  if (leftDanger && !rightDanger) turnRight(TURN_SPEED);
  else turnLeft(TURN_SPEED);
  delay(520); stopCar();
}

void avoidObstacle() {
  // 1. Stop at 5 cm, then make the first left/right scan.
  stopCar(); delay(120);
  lookAt(35);
  lookAt(145);
  scanner.write(90); delay(220);

  // 2. Reverse while the forward-facing sensor reports insufficient space.
  // The time limit prevents an endless reverse if the SRF05 has a bad echo.
  unsigned long backupStarted = millis();
  while (readDistanceCm() <= BACKUP_CLEAR_CM &&
         millis() - backupStarted < MAX_BACKUP_MS) {
    backward(AUTO_SPEED);
    delay(70);
  }
  stopCar(); delay(120);

  // 3. Scan again after backing up; choose the side with more open space.
  int leftCm = lookAt(35);
  int centreCm = lookAt(90);
  int rightCm = lookAt(145);
  scanner.write(90);
  delay(100);

  if (leftCm > rightCm && leftCm > OBSTACLE_CM) {
    turnLeft(TURN_SPEED);
    delay(480);
  } else if (rightCm > OBSTACLE_CM) {
    turnRight(TURN_SPEED);
    delay(480);
  } else if (centreCm > OBSTACLE_CM) {
    // Both sides are closed but the front became clear after reversing.
    forward(AUTO_SPEED);
    delay(220);
  } else {
    // No clear route: turn around to seek a new direction.
    turnRight(TURN_SPEED);
    delay(900);
  }
  stopCar();
}

void autonomousAvoid() {
  if (cliffAhead()) { escapeCliff(); return; }
  scanner.write(90); // look straight ahead while the car is moving forward
  int distance = readDistanceCm();
  if (distance <= OBSTACLE_CM) avoidObstacle();
  else forward(AUTO_SPEED);
}

void followObject() {
  if (cliffAhead()) { escapeCliff(); return; }

  int leftCm = lookAt(40);
  int centreCm = lookAt(90);
  int rightCm = lookAt(140);
  scanner.write(90);

  int nearest = min(centreCm, min(leftCm, rightCm));
  if (nearest > FOLLOW_MAX_CM) { stopCar(); return; } // nothing to follow

  // Turn toward the direction in which the closest object was detected.
  if (leftCm == nearest && leftCm + 8 < centreCm) {
    turnLeft(125);
  } else if (rightCm == nearest && rightCm + 8 < centreCm) {
    turnRight(125);
  } else if (centreCm > FOLLOW_MIN_CM) {
    forward(135);
  } else {
    stopCar();
  }
}

void setup() {
  pinMode(ENA, OUTPUT); pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(ENB, OUTPUT); pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);
  pinMode(CH1_PIN, INPUT); pinMode(CH2_PIN, INPUT); pinMode(CH3_PIN, INPUT);
  pinMode(TCRT_LEFT, INPUT); pinMode(TCRT_CENTRE, INPUT); pinMode(TCRT_RIGHT, INPUT);
  pinMode(TRIG_PIN, OUTPUT); pinMode(ECHO_PIN, INPUT);
  stopCar();
  scanner.attach(SERVO_PIN);
  scanner.write(90);
  lcd.begin(); lcd.backlight();
  showMode(MANUAL);
  delay(1000);
}

void loop() {
  Mode mode = readMode();
  if (mode != previousMode) {
    stopCar();
    scanner.write(90);
    manualScanAngle = 35;
    manualScanStep = 1;
    manualServoSettling = false;
    manualObstacleLocked = false;
    manualClearReads = 0;
    showMode(mode);
    previousMode = mode;
    delay(180);
  }

  if (mode == MANUAL) manualControl();
  else if (mode == AVOID) autonomousAvoid();
  else followObject();
}
