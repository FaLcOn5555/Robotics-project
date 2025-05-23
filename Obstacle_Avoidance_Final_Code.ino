#include <SoftwareSerial.h>
#include <Servo.h>
#include <NewPing.h>

// === MOTOR PINS ===
#define IN1 11
#define IN2 9
#define ENA 6
#define IN3 8
#define IN4 7
#define ENB 5

// === SENSOR PINS ===
#define Trig A5
#define Echo A4
#define irRight A2
#define irLeft A3
#define MAX_DISTANCE 300
#define OBSTACLE_DISTANCE 20

// === SERVO MOTOR ===
#define SERVO_PIN 3
Servo servo;
#define SERVO_CENTER 90
#define SERVO_LEFT 180
#define SERVO_RIGHT 0

// === MOTION TIMINGS (ms) ===
#define REVERSE_TIME 0        // You asked to keep this unchanged
#define TURN_TIME 800
#define BYPASS_TIME 500
#define FORWARD_DELAY 80

// === SPEED SETTINGS ===
#define LEFT_MOTOR_SPEED 140
#define RIGHT_MOTOR_SPEED 150
int Speed = 150;

// === STATE VARIABLES ===
int state;
int mode = 12;

NewPing sonar(Trig, Echo, MAX_DISTANCE);
SoftwareSerial BT(10, 11); // ✅ Changed to avoid Serial Monitor conflict

int distance, leftDistance, rightDistance;

void setup() {
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT); pinMode(ENB, OUTPUT);
  pinMode(Trig, OUTPUT); pinMode(Echo, INPUT);
  pinMode(irRight, INPUT); pinMode(irLeft, INPUT);

  servo.attach(SERVO_PIN);
  servo.write(SERVO_CENTER);

  Serial.begin(9600);
  BT.begin(9600);
  delay(1000);
  Serial.println("Bluetooth autonomous vehicle Ready.");
}

void loop() {
  if (BT.available() > 0) {
    state = BT.read();
    Serial.print("BT CMD: "); Serial.println(state);

    if (state == 11) {
      mode = 11; Stop();
      Serial.println("Mode: Manual Bluetooth");
    } else if (state == 12) {
      mode = 12; Stop();
      Serial.println("Mode: Obstacle Avoidance");
    }

    if (state > 21) Speed = state;
  }

  analogWrite(ENA, LEFT_MOTOR_SPEED);
  analogWrite(ENB, RIGHT_MOTOR_SPEED);

  if (mode == 11) {
    BluetoothControl();
  } else if (mode == 12) {
    ObstacleAvoid();
  }

  delay(FORWARD_DELAY);
}

// === MANUAL CONTROL ===
void BluetoothControl() {
  if (state == 1) forward();
  else if (state == 2) backward();
  else if (state == 3) turnLeft();
  else if (state == 4) turnRight();
  else if (state == 5) Stop();
}

// === OBSTACLE AVOIDANCE LOGIC ===
void ObstacleAvoid() {
  distance = ultrasonic();

  if (distance <= OBSTACLE_DISTANCE) {
    Stop(); delay(150);
    backward(); delay(REVERSE_TIME); Stop(); delay(200);

    leftDistance = readDistance(SERVO_LEFT); delay(200);
    rightDistance = readDistance(SERVO_RIGHT); delay(200);
    servo.write(SERVO_CENTER); delay(200);

    // If both directions blocked, move back and skip turn
    if (leftDistance < OBSTACLE_DISTANCE && rightDistance < OBSTACLE_DISTANCE) {
      backward(); delay(500); Stop(); delay(300);
      return;
    }

    if (leftDistance > rightDistance) {
      turnLeft(); delay(TURN_TIME); Stop(); delay(150);
      forward(); delay(BYPASS_TIME); Stop(); delay(150);
      turnRight(); delay(TURN_TIME); Stop(); delay(150);
      forward(); delay(1000); Stop(); delay(200);
      turnRight(); delay(TURN_TIME); Stop(); delay(150);
      forward(); delay(BYPASS_TIME); Stop(); delay(150);
      turnLeft(); delay(TURN_TIME); Stop(); delay(150);
    } else {
      turnRight(); delay(TURN_TIME); Stop(); delay(150);
      forward(); delay(BYPASS_TIME); Stop(); delay(150);
      turnLeft(); delay(TURN_TIME); Stop(); delay(150);
      forward(); delay(1000); Stop(); delay(200);
      turnLeft(); delay(TURN_TIME); Stop(); delay(150);
      forward(); delay(BYPASS_TIME); Stop(); delay(150);
      turnRight(); delay(TURN_TIME); Stop(); delay(150);
    }
  } else {
    forward();             // ✅ This code was previously misplaced
  }
}

// === ULTRASONIC DISTANCE MEASUREMENT ===
int ultrasonic() {
  digitalWrite(Trig, LOW); delayMicroseconds(2);
  digitalWrite(Trig, HIGH); delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  long t = pulseIn(Echo, HIGH, 30000); // ✅ Added timeout (30 ms)
  return (t * 0.0343) / 2;
}

int readDistance(int angle) {
  servo.write(angle); delay(300);
  return ultrasonic();
}

// === MOTOR MOVEMENT FUNCTIONS ===
void forward() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, LEFT_MOTOR_SPEED);
  analogWrite(ENB, RIGHT_MOTOR_SPEED);
}

void backward() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, LEFT_MOTOR_SPEED);
  analogWrite(ENB, RIGHT_MOTOR_SPEED);
}

void turnLeft() {
  digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH); digitalWrite(IN4, LOW);
  analogWrite(ENA, LEFT_MOTOR_SPEED);
  analogWrite(ENB, RIGHT_MOTOR_SPEED);
}

void turnRight() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW); digitalWrite(IN4, HIGH);
  analogWrite(ENA, LEFT_MOTOR_SPEED);
  analogWrite(ENB, RIGHT_MOTOR_SPEED);
}

void Stop() {
  digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW); digitalWrite(IN4, LOW);
}
