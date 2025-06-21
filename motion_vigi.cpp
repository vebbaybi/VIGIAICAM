#include <SoftwareSerial.h>
#include <IRremote.h>

// Pin definitions
#define TRIG_PIN 7        // Ultrasonic trigger
#define ECHO_PIN 6        // Ultrasonic echo
#define LINE_LEFT A0      // Line tracking left sensor
#define LINE_MIDDLE A1    // Line tracking middle sensor
#define LINE_RIGHT A2     // Line tracking right sensor
#define IR_PIN 12         // IR receiver
#define ENA 5             // Motor driver enable A (PWM)
#define ENB 6             // Motor driver enable B (PWM)
#define IN1 8             // Motor driver input 1
#define IN2 9             // Motor driver input 2
#define IN3 10            // Motor driver input 3
#define IN4 11            // Motor driver input 4
#define BT_RX 10          // Bluetooth TX (HC-06 RX)
#define BT_TX 11          // Bluetooth RX (HC-06 TX)

// Constants
#define SPEED 150         // Motor speed (0–255)
#define TURN_SPEED 100    // Speed for turning
#define DISTANCE_THRESHOLD 20 // Obstacle distance (cm)
#define LOOP_INTERVAL 100 // Main loop delay (ms)

// IR codes (adjust based on your remote)
#define IR_FORWARD 0x00FF629D
#define IR_BACKWARD 0x00FFA857
#define IR_LEFT 0x00FF22DD
#define IR_RIGHT 0x00FFC23D
#define IR_STOP 0x00FF02FD

// States
enum Mode { OBSTACLE_AVOID, LINE_TRACK, IR_CONTROL, BLUETOOTH };
Mode currentMode = OBSTACLE_AVOID;

// Objects
SoftwareSerial btSerial(BT_RX, BT_TX); // Bluetooth serial
IRrecv irrecv(IR_PIN);                 // IR receiver
decode_results results;                // IR decode results

void setup() {
  // Initialize Serial for debugging
  Serial.begin(9600);
  btSerial.begin(9600); // Bluetooth
  irrecv.enableIRIn(); // Start IR receiver

  // Set pin modes
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LINE_LEFT, INPUT);
  pinMode(LINE_MIDDLE, INPUT);
  pinMode(LINE_RIGHT, INPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.println("Robot Car Ready");
}

void loop() {
  switch (currentMode) {
    case OBSTACLE_AVOID:
      obstacleAvoid();
      break;
    case LINE_TRACK:
      lineTrack();
      break;
    case IR_CONTROL:
      irControl();
      break;
    case BLUETOOTH:
      bluetoothControl();
      break;
  }

  // Check for mode switch (via Bluetooth or Serial)
  if (btSerial.available()) {
    char cmd = btSerial.read();
    switchMode(cmd);
  }
  if (Serial.available()) {
    char cmd = Serial.read();
    switchMode(cmd);
  }

  delay(LOOP_INTERVAL);
}

// Motor control functions
void moveForward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void moveBackward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, SPEED);
  analogWrite(ENB, SPEED);
}

void turnLeft() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void turnRight() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, TURN_SPEED);
  analogWrite(ENB, TURN_SPEED);
}

void stopMotors() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

// Ultrasonic distance measurement
long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duration = pulseIn(ECHO_PIN, HIGH);
  return duration * 0.034 / 2; // Distance in cm
}

// Obstacle avoidance
void obstacleAvoid() {
  long distance = getDistance();
  Serial.print("Distance: ");
  Serial.println(distance);

  if (distance < DISTANCE_THRESHOLD) {
    stopMotors();
    delay(200);
    moveBackward();
    delay(500);
    turnRight();
    delay(300);
  } else {
    moveForward();
  }
}

// Line tracking
void lineTrack() {
  bool left = digitalRead(LINE_LEFT);
  bool middle = digitalRead(LINE_MIDDLE);
  bool right = digitalRead(LINE_RIGHT);

  if (middle) {
    moveForward();
  } else if (left) {
    turnLeft();
  } else if (right) {
    turnRight();
  } else {
    stopMotors();
  }
}

// IR remote control
void irControl() {
  if (irrecv.decode(&results)) {
    Serial.println(results.value, HEX);
    switch (results.value) {
      case IR_FORWARD:
        moveForward();
        break;
      case IR_BACKWARD:
        moveBackward();
        break;
      case IR_LEFT:
        turnLeft();
        break;
      case IR_RIGHT:
        turnRight();
        break;
      case IR_STOP:
        stopMotors();
        break;
    }
    irrecv.resume();
  }
}

// Bluetooth control
void bluetoothControl() {
  if (btSerial.available()) {
    char cmd = btSerial.read();
    Serial.println(cmd);
    switch (cmd) {
      case 'F':
        moveForward();
        break;
      case 'B':
        moveBackward();
        break;
      case 'L':
        turnLeft();
        break;
      case 'R':
        turnRight();
        break;
      case 'S':
        stopMotors();
        break;
    }
  }
}

// Mode switching
void switchMode(char cmd) {
  switch (cmd) {
    case '1':
      currentMode = OBSTACLE_AVOID;
      Serial.println("Mode: Obstacle Avoidance");
      break;
    case '2':
      currentMode = LINE_TRACK;
      Serial.println("Mode: Line Tracking");
      break;
    case '3':
      currentMode = IR_CONTROL;
      Serial.println("Mode: IR Control");
      break;
    case '4':
      currentMode = BLUETOOTH;
      Serial.println("Mode: Bluetooth Control");
      break;
  }
}
