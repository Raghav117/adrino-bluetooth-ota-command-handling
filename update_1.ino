// This is the new code you want to run on your ESP32.
// It will make the built-in LED blink.

// Define the pin for the built-in LED (usually GPIO 2 on most ESP32 boards)

#include <Arduino.h>
#include "BLEOtaHandler.h"
#include "CommandHandler.h"
#include <Update.h>

BLEOtaHandler otaHandler;
CommandHandler commandHandler; 


// --- Main Setup Function ---
void setup() {
  Serial.begin(115200);
  commandHandler.begin();
  otaHandler.begin("good");
  otaHandler.setCommandHandler(&commandHandler);


  setup_1();
}


// --- Main Loop ---
void loop() {
  // The loop can be empty. All work is done in the BLE callbacks.
  // A small delay can help the ESP32 yield to other tasks if needed.
  loop_1();
}

// --- Motor Speed Control Pins (PWM) ---
#define LEFT_MOTOR_ENABLE 13 // Connect to ENA on L298N
#define RIGHT_MOTOR_ENABLE 15 // Connect to ENB on L298N

// --- PWM Configuration ---
#define PWM_FREQUENCY 5000     // 5 kHz
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define PWM_RESOLUTION 8       // 8-bit resolution (0-255)

int speed = 200;

#define LED_PIN 2
#define LEFT_MOTOR_IN1 26 // Connect to IN1 on L298N
#define LEFT_MOTOR_IN2 25 // Connect to IN2 on L298N

// --- Right Motor Direction Pins ---
#define RIGHT_MOTOR_IN3 12 // Connect to IN3 on L298N
#define RIGHT_MOTOR_IN4 14 // Connect to IN4 on L298N


void moveForward() {
  Serial.println("Action: Moving Forward");
  // Left Motor Forward
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  // Right Motor Forward
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

   // Set speed
  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}


void moveBackward() {
  Serial.println("Action: Moving Backward");
  // Left Motor Backward
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  // Right Motor Backward
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);

   // Set speed
  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnLeft() {
  Serial.println("Action: Turning Left");
  // Left Motor Backward (to pivot)
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  // Right Motor Forward
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

   // Set speed
  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void turnRight() {
  Serial.println("Action: Turning Right");
  // Left Motor Forward
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  // Right Motor Backward (to pivot)
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);

   // Set speed
  ledcWrite(PWM_CHANNEL_LEFT, speed);
  ledcWrite(PWM_CHANNEL_RIGHT, speed);
}

void stopMotors() {
  Serial.println("Action: Stopping");
  // By setting both input pins to LOW, the motor driver stops the motors.
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);

   // Set speed
  ledcWrite(PWM_CHANNEL_LEFT, 0);
  ledcWrite(PWM_CHANNEL_RIGHT, 0);
}

void setupPWM() {
  // Configure the PWM channels
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);

  // Attach the GPIO pins to the PWM channels
  ledcAttachPin(LEFT_MOTOR_ENABLE, PWM_CHANNEL_LEFT);
  ledcAttachPin(RIGHT_MOTOR_ENABLE, PWM_CHANNEL_RIGHT);
  
  Serial.println("PWM has been configured.");
}

void setup_1() {
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  // Set the LED pin as an output
  pinMode(LED_PIN, OUTPUT);

  setupPWM();


  stopMotors();

}

void blinkLED(int times) {
  for (int i = 0; i < times; ++i) {
    digitalWrite(2, HIGH);
    delay(200);
    digitalWrite(2, LOW);
    delay(200);
  }
}

void loop_1() {

  if (commandHandler.hasNewCommand()) {
    String cmd = commandHandler.getLastCommand();
    Serial.println("[CMD] Received: " + cmd);
    blinkLED(1);

// Check if the command is for setting speed
    if (cmd.startsWith("SPEED_")) {
      // Extract the speed value from the command string
      String speedValue = cmd.substring(6);
      speed = speedValue.toInt();
      // Constrain the value to be within the valid PWM range (0-255)
      speed = constrain(speed, 0, 255);
      Serial.print("Action: Speed set to ");
      Serial.println(speed);
    }

     else if (cmd == "UP") {
      moveBackward();
      
    } else if (cmd == "DOWN") {
      moveForward();
    } else if (cmd == "LEFT") {
      turnLeft();
    } else if (cmd == "RIGHT") {
      turnRight();
    } else if (cmd=="STOP"){
      stopMotors();
    }
    

  
    }
  }
 
