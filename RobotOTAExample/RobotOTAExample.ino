/*
  Robot BLE OTA Update Example
  
  This example integrates the BLE OTA Update library with robot control functionality.
  It demonstrates how to use OTA updates while maintaining robot control capabilities.
  
  Based on the original robot control code with added BLE OTA functionality.
*/

#include <BLEOtaUpdate.h>

// Custom UUIDs from original BLEOtaHandler
const char* CUSTOM_SERVICE_UUID = "66443771-D481-49B0-BE32-8CE24AC0F09C";
const char* CUSTOM_OTA_CHAR_UUID = "66443772-D481-49B0-BE32-8CE24AC0F09C";
const char* CUSTOM_COMMAND_CHAR_UUID = "66443773-D481-49B0-BE32-8CE24AC0F09C";
const char* CUSTOM_STATUS_CHAR_UUID = "12233448-D481-49B0-BE32-8CE24AC0F09C";

// Create BLE OTA instance with custom UUIDs
BLEOtaUpdate bleOta(CUSTOM_SERVICE_UUID, CUSTOM_OTA_CHAR_UUID, CUSTOM_COMMAND_CHAR_UUID, CUSTOM_STATUS_CHAR_UUID);

// Motor Speed Control Pins (PWM)
#define LEFT_MOTOR_ENABLE 13
#define RIGHT_MOTOR_ENABLE 15

// PWM Configuration
#define PWM_FREQUENCY 5000
#define PWM_CHANNEL_LEFT 0
#define PWM_CHANNEL_RIGHT 1
#define PWM_RESOLUTION 8

// Motor Direction Pins
#define LEFT_MOTOR_IN1 26
#define LEFT_MOTOR_IN2 25
#define RIGHT_MOTOR_IN3 12
#define RIGHT_MOTOR_IN4 14

// LED pin for status indication
#define LED_PIN 2

// Infrared Sensor
#define LEFT_SENSOR 35
#define RIGHT_SENSOR 34

// Ultrasonic Sensor
#define TRIG_PIN 2
#define ECHO_PIN 4

// Moving Direction
enum class MovingDirection {
  UP,
  DOWN,
  LEFT,
  RIGHT,
  STOP
};

MovingDirection movingDirection;
int speed = 100;
int dist = 999;
bool isUltrasonicEnable = false;
int ultrasonicRange = 40;
unsigned long reverseStartTime = 0;
bool reversing = false;


bool isInfraredEnable = false;






long getDistance();

void infraredLoop();

bool ultrasonicLoop();

bool ultrasonicLoop(){
  if(isUltrasonicEnable==false){
    return false;
  }
  bool isUnderRange = false;
  dist = getDistance();
  if(!reversing && dist<=ultrasonicRange && movingDirection == MovingDirection::UP){
    isUnderRange = true;
    stopMotors();
    moveDown();
    reversing = true; // start reversing mode
    reverseStartTime = millis(); // mark the start time
    stopMotors();
  }
  if(reversing){
        if(millis() - reverseStartTime >= 300){ // 400 ms passed
            stopMotors();
            reversing = false; // end reversing mode
        }
    }
  Serial.print(" | Ultrasonic: "); Serial.print(dist);
  Serial.println("cm");
  return isUnderRange;
}


void infraredLoop(){
  if(isInfraredEnable==false){
    return;
  }
  int x = analogRead(RIGHT_SENSOR);
  
  int y = analogRead(LEFT_SENSOR);
  Serial.printf("infrared values: %d %d \n", x,y);

  int threshold = 1000;

   if (x < threshold && y >= threshold) {
    turnLeft();
    delay(400);
  } 
  else if (y < threshold && x >= threshold) {
    turnRight();
    delay(400);
  } else if(x>=threshold and y>=threshold){
    stopMotors();
  }
  else {
    moveUP();
  }
}


long getDistance() {
// Send a 10us pulse to trigger
digitalWrite(TRIG_PIN, LOW);
delayMicroseconds(2);
digitalWrite(TRIG_PIN, HIGH);
delayMicroseconds(10);
digitalWrite(TRIG_PIN, LOW);
// measure echo time
long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // 30ms timeout
// Convert to centimeters
long distance = duration * 0.0343 / 2;
if (distance == 0) distance = 999; // if no echo, treat as "far away"
return distance;
}


void setup() {
  Serial.begin(115200);

  pinMode(LEFT_SENSOR, INPUT);
  pinMode(RIGHT_SENSOR, INPUT);
  
  // Configure motor pins
   pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  
  
  // Setup PWM
  setupPWM();
  
  // Stop motors initially
  stopMotors();
  
  Serial.println("Starting Robot BLE OTA Example with Custom UUIDs...");
  Serial.println("Using custom UUIDs:");
  Serial.printf("Service UUID: %s\n", CUSTOM_SERVICE_UUID);
  Serial.printf("OTA Characteristic UUID: %s\n", CUSTOM_OTA_CHAR_UUID);
  Serial.printf("Command Characteristic UUID: %s\n", CUSTOM_COMMAND_CHAR_UUID);
  Serial.printf("Status Characteristic UUID: %s\n", CUSTOM_STATUS_CHAR_UUID);
  
  // Set up BLE OTA callbacks
  bleOta.setOtaProgressCallback(onOtaProgress);
  bleOta.setOtaStatusCallback(onOtaStatus);
  bleOta.setCommandCallback(onCommand);
  bleOta.setConnectionCallback(onConnection);
  
  // Start BLE OTA service
  bleOta.begin("Car Robot-ESP32");
  
  Serial.println("Robot BLE OTA service is ready!");
  Serial.println("Available commands:");
  Serial.println("  UP/DOWN/LEFT/RIGHT/STOP - Robot movement");
  Serial.println("  SPEED_XXX - Set speed (0-255)");
  Serial.println("  ENABLE/DISABLE - Enable/disable robot");
  Serial.println("  STATUS - Get robot status");
}

void loop() {
  infraredLoop();
  
  ultrasonicLoop();
 }

// BLE OTA Callbacks
void onOtaProgress(uint32_t received, uint32_t total, uint8_t percentage) {
  Serial.printf("OTA Progress: %d%% (%u/%u bytes)\n", percentage, received, total);
  
  // Visual feedback with LED
  if (percentage % 10 == 0) {
    digitalWrite(LED_PIN, HIGH);
    delay(50);
    digitalWrite(LED_PIN, LOW);
  }
}

void onOtaStatus(OtaStatus status, const char* message) {
  Serial.printf("OTA Status: %s\n", message);
  
  switch (status) {
    case OtaStatus::IDLE:
      digitalWrite(LED_PIN, LOW);
      break;
    case OtaStatus::RECEIVING:
      digitalWrite(LED_PIN, HIGH);
      break;
    case OtaStatus::COMPLETED:
      // LED will stay on until reboot
      break;
    case OtaStatus::ERROR:
      // Blink rapidly to indicate error
      for (int i = 0; i < 5; i++) {
        digitalWrite(LED_PIN, HIGH);
        delay(100);
        digitalWrite(LED_PIN, LOW);
        delay(100);
      }
      break;
    case OtaStatus::ABORTED:
      digitalWrite(LED_PIN, LOW);
      break;
  }
}

void onCommand(const String& command) {
  Serial.printf("Received command: %s\n", command.c_str());

  
  // Robot movement commands
  if (command == "UP") {
    moveUP();
    bleOta.sendStatus("Moving forward");
  } else if (command == "DOWN") {
    moveDown();
    bleOta.sendStatus("Moving backward");
  } else if (command == "LEFT") {
    turnLeft();
    bleOta.sendStatus("Turning left");
  } else if (command == "RIGHT") {
    turnRight();
    bleOta.sendStatus("Turning right");
  } else if (command == "STOP") {
    stopMotors();
    bleOta.sendStatus("Stopped");
  } else if (command.startsWith("SPEED_")) {
    String speedValue = command.substring(6);
    speed = speedValue.toInt();
    speed = constrain(speed, 0, 255);
    bleOta.sendStatus("Speed set to " + String(speed));
  } else if(command.startsWith("ULTRASONIC_")) {
      String ultrasonicValue = command.substring(11);
      if(ultrasonicValue.startsWith("OFF")){
        isUltrasonicEnable=false;
      }else{
        ultrasonicRange = ultrasonicValue.toInt();
        ultrasonicRange = constrain(ultrasonicRange, 10, 100);
        bleOta.sendStatus("ultrasonicRange set to " + String(ultrasonicRange));
        if(isUltrasonicEnable==false){
          isUltrasonicEnable=true;
        }
      }
  } else if(command.startsWith("INFRARED_")){
      String infraredValue = command.substring(9);
      if(infraredValue.startsWith("ON")){
        isInfraredEnable=true;
      }else if(infraredValue.startsWith("OFF")){
        stopMotors();
        isInfraredEnable=false;
      }
  } 
  else {
    bleOta.sendStatus("Unknown command: " + command);
  }
}

void onConnection(bool connected) {
  if (connected) {
    Serial.println("BLE client connected");
    digitalWrite(LED_PIN, HIGH);
    bleOta.sendStatus("Robot connected and ready!");
  } else {
    Serial.println("BLE client disconnected");
    digitalWrite(LED_PIN, LOW);
  }
}

// Robot movement functions
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

void moveUP() {

   bool isDistanceUnderRange = ultrasonicLoop();
   if(isDistanceUnderRange){
    return;
   }

  movingDirection = MovingDirection::UP;

  
  Serial.println("Action: Moving UP");
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  
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

void setupPWM() {
  ledcSetup(PWM_CHANNEL_LEFT, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(PWM_CHANNEL_RIGHT, PWM_FREQUENCY, PWM_RESOLUTION);
  
  ledcAttachPin(LEFT_MOTOR_ENABLE, PWM_CHANNEL_LEFT);
  ledcAttachPin(RIGHT_MOTOR_ENABLE, PWM_CHANNEL_RIGHT);
  
  Serial.println("PWM configured for motor control");
}
