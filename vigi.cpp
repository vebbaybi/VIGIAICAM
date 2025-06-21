#include <Servo.h>
#include <SoftwareSerial.h>

// =============================
// === PIN ASSIGNMENTS ===
// =============================
#define PIN_PIR        2
#define PIN_TRIG1      3
#define PIN_ECHO1      4
#define PIN_SERVO      5
#define LED_PIN        6
#define PIN_TRIG2      8
#define PIN_ECHO2      9
#define SERIAL_RX     10
#define SERIAL_TX     11

// =============================
// === SYSTEM CONSTANTS ===
// =============================
#define MAIN_LOOP_INTERVAL_MS     200
#define PATROL_INTERVAL_MS        5000
#define WATCHDOG_TIMEOUT_MS       2000
#define THREAT_SCORE_ALERT_THRESHOLD  70
#define DISTANCE_THRESHOLD_M      1.5
#define CONFIDENCE_THRESHOLD      80
#define SERVO_MIN_ANGLE           0
#define SERVO_MAX_ANGLE           180
#define SERVO_STEP_ANGLE          15

// =============================
// === GLOBAL VARIABLES ===
// =============================
Servo panServo;
SoftwareSerial espSerial(SERIAL_RX, SERIAL_TX);

bool motionDetected = false;
float distance = 0.0;
float confidence = 0.0;
bool sensorValid = false;

float dist1 = -1;
float dist2 = -1;

int currentState = 0; // 0 = IDLE, 1 = PATROLLING, 2 = ALERT, 3 = FAULT
unsigned long lastLoopTime = 0;
unsigned long lastPatrolTime = 0;
unsigned long lastWatchdogFeed = 0;
int currentServoPos = 90;
bool isInitialized = false;

// Alert blink control
bool alertBlinkState = false;
unsigned long lastAlertBlinkTime = 0;

// Fault blinking
bool faultBlinkState = false;
unsigned long lastFaultBlinkTime = 0;

// =============================
// === LOGGING FUNCTION ===
// =============================
void logMessage(const char* level, const char* message) {
  Serial.print("[" + String(millis() / 1000.0, 1) + "s ");
  Serial.print(level);
  Serial.print("] ");
  Serial.println(message);
}

// =============================
// === WATCHDOG ===
// =============================
void feedWatchdog() {
  lastWatchdogFeed = millis();
}

bool checkWatchdog() {
  if (millis() - lastWatchdogFeed > WATCHDOG_TIMEOUT_MS) {
    logMessage("ERROR", "Watchdog timeout!");
    currentState = 3;
    return false;
  }
  return true;
}

// =============================
// === INITIALIZATION ===
// =============================
void initializeSystem() {
  Serial.begin(9600);
  espSerial.begin(4800);

  pinMode(PIN_PIR, INPUT);
  pinMode(PIN_TRIG1, OUTPUT);
  pinMode(PIN_ECHO1, INPUT);
  pinMode(PIN_TRIG2, OUTPUT);
  pinMode(PIN_ECHO2, INPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);
  panServo.attach(PIN_SERVO);
  panServo.write(currentServoPos);

  logMessage("INFO", "System initialized");
  isInitialized = true;
  feedWatchdog();
}

// =============================
// === SENSOR UPDATE ===
// =============================
float readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 20000); // max 20ms = ~3.4m
  if (duration == 0) return -1;
  return (duration / 2.0) * 0.0343; // in cm
}

void updateSensors() {
  motionDetected = digitalRead(PIN_PIR) == HIGH;

  dist1 = readUltrasonicCM(PIN_TRIG1, PIN_ECHO1);
  dist2 = readUltrasonicCM(PIN_TRIG2, PIN_ECHO2);

  if (dist1 < 0 && dist2 < 0) {
    sensorValid = false;
    logMessage("WARN", "Ultrasonic read failed");
    return;
  }

  distance = min(dist1, dist2) / 100.0;
  confidence = (distance > 0 && distance < DISTANCE_THRESHOLD_M) ? 90.0 : 40.0;
  sensorValid = true;

  Serial.print("DEBUG Sensors - Motion: ");
  Serial.print(motionDetected);
  Serial.print(" Dist: ");
  Serial.print(distance, 2);
  Serial.print("m Conf: ");
  Serial.println(confidence, 1);
}

// =============================
// === THREAT SCORING ===
// =============================
int calculateThreatScore() {
  if (!sensorValid) return 0;

  int score = 0;
  if (motionDetected) score += 40;
  if (distance < DISTANCE_THRESHOLD_M) score += 30 * (1.0 - distance / DISTANCE_THRESHOLD_M);
  if (confidence > CONFIDENCE_THRESHOLD) score += 30 * (confidence / 100.0);
  return min(score, 100);
}

// =============================
// === SERVO ACTION ===
// =============================
void moveServoSmooth(int targetPos) {
  targetPos = constrain(targetPos, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
  while (abs(currentServoPos - targetPos) > SERVO_STEP_ANGLE) {
    currentServoPos += (targetPos > currentServoPos) ? SERVO_STEP_ANGLE : -SERVO_STEP_ANGLE;
    panServo.write(currentServoPos);
    delay(20);
  }
  currentServoPos = targetPos;
  panServo.write(currentServoPos);
}

// =============================
// === LED ALERT (DYNAMIC) ===
// =============================
void handleAlertBlink() {
  if (!motionDetected || !sensorValid || distance <= 0.0) {
    digitalWrite(LED_PIN, LOW);
    return;
  }

  unsigned long blinkInterval = map(constrain(distance * 100, 20, 150), 20, 150, 50, 300);

  if (millis() - lastAlertBlinkTime >= blinkInterval) {
    alertBlinkState = !alertBlinkState;
    digitalWrite(LED_PIN, alertBlinkState ? HIGH : LOW);
    lastAlertBlinkTime = millis();
  }
}

void handleFaultBlink() {
  if (millis() - lastFaultBlinkTime >= 500) {
    faultBlinkState = !faultBlinkState;
    digitalWrite(LED_PIN, faultBlinkState ? HIGH : LOW);
    lastFaultBlinkTime = millis();
  }
}

// =============================
// === PATROLLING ===
// =============================
void executePatrol() {
  int nextPos = currentServoPos + 45;
  if (nextPos > SERVO_MAX_ANGLE) nextPos = SERVO_MIN_ANGLE;
  moveServoSmooth(nextPos);
  lastPatrolTime = millis();
}

// =============================
// === OTA & COMMS ===
// =============================
void checkOTA() {
  static int count = 0;
  if (count++ % 5 == 0) {
    logMessage("INFO", "Checking for OTA (mock)");
  }
}

void handleSerialComms() {
  if (espSerial.available()) {
    String input = espSerial.readStringUntil('\n');
    Serial.print("Serial RX: ");
    Serial.println(input);
  }
}

void sendCameraCommand(const char* cmd) {
  espSerial.println(cmd);
  Serial.print("ESP32-CAM Command Sent: ");
  Serial.println(cmd);
}

// =============================
// === STATE MACHINE ===
// =============================
void runStateMachine() {
  feedWatchdog();
  if (!checkWatchdog()) return;

  switch (currentState) {
    case 0: {
      updateSensors();
      if (sensorValid) {
        int score = calculateThreatScore();
        Serial.print("Threat score: ");
        Serial.println(score);
        if (score >= THREAT_SCORE_ALERT_THRESHOLD) {
          currentState = 2;
          // Directional servo logic
          if (dist1 > 0 && dist2 > 0) {
            moveServoSmooth((dist1 < dist2) ? 45 : 135);
          }
          sendCameraCommand("CAM_CAPTURE");
        } else if (millis() - lastPatrolTime > PATROL_INTERVAL_MS) {
          currentState = 1;
        }
      }
      break;
    }
    case 1:
      executePatrol();
      currentState = 0;
      break;
    case 2:
      updateSensors();
      handleAlertBlink();
      if (!motionDetected) currentState = 0;
      break;
    case 3:
      handleFaultBlink();
      break;
  }

  checkOTA();
  handleSerialComms();
}

// =============================
// === MAIN LOOP ===
// =============================
void setup() {
  initializeSystem();
}

void loop() {
  if (!isInitialized) return;

  if (millis() - lastLoopTime >= MAIN_LOOP_INTERVAL_MS) {
    runStateMachine();
    lastLoopTime = millis();
  }
}
