// RobotHardware.h
#ifndef ROBOT_HARDWARE_H
#define ROBOT_HARDWARE_H

#include <Arduino.h>

// ===== Pins =====
#define LEFT_MOTOR_ENABLE 13
#define RIGHT_MOTOR_ENABLE 15

#define LEFT_MOTOR_IN1 26
#define LEFT_MOTOR_IN2 25
#define RIGHT_MOTOR_IN3 12
#define RIGHT_MOTOR_IN4 14

#define LED_PIN 2
#define LEFT_SENSOR 35
#define RIGHT_SENSOR 34
#define TRIG_PIN 2
#define ECHO_PIN 4

// ===== PWM =====
#define PWM_FREQUENCY 5000
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define PWM_RESOLUTION 8

// ===== Direction Enum =====
enum class MovingDirection {
  UP,
  DOWN,
  LEFT,
  RIGHT,
  STOP
};

// ===== Global States =====
extern MovingDirection movingDirection;
extern int speed;
extern int dist;
extern bool isUltrasonicEnable;
extern int ultrasonicRange;
extern bool isInfraredEnable;
extern bool reversing;

// ===== Hardware Functions =====
void hardwareSetup();
void setupPWM();

void moveUP();
void moveDown();
void turnLeft();
void turnRight();
void stopMotors();

long getDistance();
bool ultrasonicLoop();
void infraredLoop();
void hardwareLoop();

// Command handler for BLE
String onCommand(const String& command);


#endif
