// RobotHardware.cpp
#include "RobotHardware.h"

// ===== Global Variables =====
MovingDirection movingDirection = MovingDirection::STOP;
int speed = 100;
int dist = 999;
bool isUltrasonicEnable = false;
int ultrasonicRange = 40;
bool isInfraredEnable = false;
bool reversing = false;
unsigned long reverseStartTime = 0;

void hardwareSetup() {
  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);

  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  setupPWM();
  stopMotors();
  Serial.println("Hardware setup completed.");
}

void hardwareLoop(){
  infraredLoop();
  ultrasonicLoop();
  
}

void setupPWM() {
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);

  ledcAttachPin(LEFT_MOTOR_ENABLE, PWM_CHANNEL_LEFT);
  ledcAttachPin(RIGHT_MOTOR_ENABLE, PWM_CHANNEL_RIGHT);

  Serial.println("PWM configured for motor control");
}

void moveUP() {
  if (ultrasonicLoop()) return; // safety stop

  movingDirection = MovingDirection::UP;
  Serial.println("Action: Moving UP");
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);

  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void moveDown() {
  movingDirection = MovingDirection::DOWN;
  Serial.println("Action: Moving Down");
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnLeft() {
  movingDirection = MovingDirection::LEFT;
  Serial.println("Action: Turning Left");
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnRight() {
  movingDirection = MovingDirection::RIGHT;
  Serial.println("Action: Turning Right");
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);

  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void stopMotors() {
  movingDirection = MovingDirection::STOP;
  Serial.println("Action: Stopping");
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

long getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  long distance = duration * 0.0343 / 2;
  if (distance == 0) distance = 999;
  return distance;
}

bool ultrasonicLoop() {
  if (!isUltrasonicEnable) return false;

  bool isUnderRange = false;
  dist = getDistance();

  if (!reversing && dist <= ultrasonicRange && movingDirection == MovingDirection::UP) {
    isUnderRange = true;
    stopMotors();
    moveDown();
    reversing = true;
    reverseStartTime = millis();
    stopMotors();
  }

  if (reversing) {
    if (millis() - reverseStartTime >= 300) {
      stopMotors();
      reversing = false;
    }
  }

  Serial.printf(" | Ultrasonic: %d cm\n", dist);
  return isUnderRange;
}

void infraredLoop() {
  if (!isInfraredEnable) return;

  int x = analogRead(RIGHT_SENSOR);
  int y = analogRead(LEFT_SENSOR);
  Serial.printf("Infrared values: %d %d\n", x, y);

  int threshold = 1000;
  if (x < threshold && y >= threshold) {
    turnLeft();
    delay(400);
  } else if (y < threshold && x >= threshold) {
    turnRight();
    delay(400);
  } else if (x >= threshold && y >= threshold) {
    stopMotors();
  } else {
    moveUP();
  }
}

String onCommand(const String& command) {
  Serial.printf("Received command: %s\n", command.c_str());

  if (command == "UP") {
    moveUP();
    return "Moving forward";
  } else if (command == "DOWN") {
    moveDown();
    return "Moving backward";
  } else if (command == "LEFT") {
    turnLeft();
    return "Turning left";
  } else if (command == "RIGHT") {
    turnRight();
    return "Turning right";
  } else if (command == "STOP") {
    stopMotors();
    return "Stopped";
  } else if (command.startsWith("SPEED_")) {
    String speedValue = command.substring(6);
    speed = speedValue.toInt();
    speed = constrain(speed, 0, 255);
    return "Speed set to " + String(speed);
  } else if (command.startsWith("ULTRASONIC_")) {
    String ultrasonicValue = command.substring(11);
    if (ultrasonicValue.startsWith("OFF")) {
      isUltrasonicEnable = false;
      return "Ultrasonic disabled";
    } else {
      ultrasonicRange = ultrasonicValue.toInt();
      ultrasonicRange = constrain(ultrasonicRange, 10, 100);
      isUltrasonicEnable = true;
      return "Ultrasonic range set to " + String(ultrasonicRange);
    }
  } else if (command.startsWith("INFRARED_")) {
    String infraredValue = command.substring(9);
    if (infraredValue.startsWith("ON")) {
      isInfraredEnable = true;
      return "Infrared enabled";
    } else if (infraredValue.startsWith("OFF")) {
      stopMotors();
      isInfraredEnable = false;
      return "Infrared disabled";
    }
  }

  return "Unknown command: " + command;
}
