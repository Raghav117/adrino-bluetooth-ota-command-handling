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
  otaHandler.begin("hlop");
  otaHandler.setCommandHandler(&commandHandler);


  setup_1();
}


// --- Main Loop ---
void loop() {
  // The loop can be empty. All work is done in the BLE callbacks.
  // A small delay can help the ESP32 yield to other tasks if needed.
  loop_1();
}
#define LED_PIN 2

void setup_1() {
  // Set the LED pin as an output
  pinMode(LED_PIN, OUTPUT);
}

void loop_1() {

  if (commandHandler.hasNewCommand()) {
    String cmd = commandHandler.getLastCommand();
    Serial.println("[CMD] Received: " + cmd);

    if (cmd == "A") {
       digitalWrite(LED_PIN, HIGH);
  delay(1000); // Wait for one second

  // Turn the LED off
  digitalWrite(LED_PIN, LOW);
  delay(1000); // Wait for one second
    } else if (cmd == "B") {
       digitalWrite(LED_PIN, HIGH);
  delay(1000); // Wait for one second

  // Turn the LED off
  digitalWrite(LED_PIN, LOW);
  delay(1000); // Wait for one second
  
       digitalWrite(LED_PIN, HIGH);
  delay(1000); // Wait for one second

  // Turn the LED off
  digitalWrite(LED_PIN, LOW);
  delay(1000); // Wait for one second
    } else {
      Serial.println("[CMD] Unhandled command: " + cmd);
    }
  }

  delay(10); 
  // Turn the LED on
 
}
