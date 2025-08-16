#include <BLEOtaUpdate.h>
#include "RobotHardware.h"

// Custom UUIDs from original BLEOtaHandler
const char* CUSTOM_SERVICE_UUID       = "88881231-A981-99B0-BA32-1BD54A51B97C";
const char* CUSTOM_OTA_CHAR_UUID      = "88881232-A981-99B0-BA32-1BD54A51B97C";
const char* CUSTOM_COMMAND_CHAR_UUID  = "88881233-A981-99B0-BA32-1BD54A51B97C";
const char* CUSTOM_STATUS_CHAR_UUID   = "88881234-A981-99B0-BA32-1BD54A51B97C";


// Create BLE OTA instance with custom UUIDs
BLEOtaUpdate bleOta(CUSTOM_SERVICE_UUID, CUSTOM_OTA_CHAR_UUID, CUSTOM_COMMAND_CHAR_UUID, CUSTOM_STATUS_CHAR_UUID);

void setup() {
  Serial.begin(115200);
  hardwareSetup();


  // Set up BLE OTA callbacks
  bleOta.setOtaProgressCallback(onOtaProgress);
  bleOta.setOtaStatusCallback(onOtaStatus);
  
  bleOta.setCommandCallback([](const String& command) {
    String result = onCommand(command);   // hardware handles it
    bleOta.sendStatus(result);            // BLE layer sends back
  });
  
  bleOta.setConnectionCallback(onConnection);
  
  // Start BLE OTA service
  bleOta.begin("Car Robot-ESP32");
}

void loop() {
  hardwareLoop();
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
