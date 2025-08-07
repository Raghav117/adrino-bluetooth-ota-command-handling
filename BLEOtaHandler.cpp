#include "BLEOtaHandler.h"

#define SERVICE_UUID        "66443771-D481-49B0-BE32-8CE24AC0F09C"
#define CHARACTERISTIC_UUID "66443772-D481-49B0-BE32-8CE24AC0F09C"

BLEOtaHandler* BLEOtaHandler::instance = nullptr;

void BLEOtaHandler::begin(const char* deviceName) {
  instance = this;

  BLEDevice::init(deviceName);

  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());

  // --- Additional Characteristic for Commands ---
  BLECharacteristic *pCommandCharacteristic = pService->createCharacteristic(
    "66443773-D481-49B0-BE32-8CE24AC0F09C", // New UUID
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_WRITE_NR
  );
  
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());



  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("[INFO] BLE OTA Handler Ready.");
}

// --- Server Callback Wrappers ---
void BLEOtaHandler::MyServerCallbacks::onConnect(BLEServer* pServer) {
  if (instance) instance->onClientConnect();
}
void BLEOtaHandler::MyServerCallbacks::onDisconnect(BLEServer* pServer) {
  if (instance) instance->onClientDisconnect();
}

// --- Characteristic Callback Wrapper ---
void BLEOtaHandler::MyCallbacks::onWrite(BLECharacteristic *pCharacteristic) {
  if (instance) instance->handleWrite(pCharacteristic);
}

// --- Logic Implementations ---
void BLEOtaHandler::onClientConnect() {
  Serial.println("[BLE] Client Connected.");
}

void BLEOtaHandler::onClientDisconnect() {
  if (otaInProgress) {
    Serial.println("[OTA] Error: Client disconnected. Aborting...");
    Update.end(false);
    otaInProgress = false;
  }
  Serial.println("[BLE] Client Disconnected. Re-advertising...");
  BLEDevice::startAdvertising();
}

void BLEOtaHandler::handleWrite(BLECharacteristic *pCharacteristic) {
  std::string value = pCharacteristic->getValue();
  const uint8_t* data = (const uint8_t*)value.c_str();
  size_t length = value.length();

  if (length == 0) return;

  if (!otaInProgress && length == 4 && memcmp(data, "OPEN", 4) == 0) {
    Serial.println("[OTA] Update started.");
    otaInProgress = true;
    otaFileSize = 0;
    otaReceived = 0;
    return;
  }

  if (otaInProgress) {
    if (otaFileSize == 0 && length == 4) {
      memcpy(&otaFileSize, data, 4);
      Serial.printf("[OTA] Update size: %u bytes.\n", otaFileSize);
      if (!Update.begin(UPDATE_SIZE_UNKNOWN)) {
        Serial.println("[OTA] ERROR: Not enough space.");
        otaInProgress = false;
      }
      return;
    }

    if (length == 4 && memcmp(data, "DONE", 4) == 0) {
      Serial.println("[OTA] Finalizing update...");
      if (otaReceived != otaFileSize) {
        Serial.printf("[OTA] ERROR: Size mismatch! (%u/%u)\n", otaReceived, otaFileSize);
        Update.end(false);
      } else if (Update.end(true)) {
        Serial.println("[OTA] Success. Rebooting...");
        ESP.restart();
      } else {
        Serial.println("[OTA] Finalize failed.");
        Update.printError(Serial);
      }
      otaInProgress = false;
      return;
    }

    if (otaReceived < otaFileSize) {
      if (Update.write((uint8_t*)data, length) > 0) {
        otaReceived += length;
        int progress = (otaReceived * 100) / otaFileSize;
        Serial.printf("[OTA] Progress: %d%%\r", progress);
      }
    }
  }
}

void BLEOtaHandler::setCommandHandler(CommandHandler* handler) {
  commandHandler = handler;
}

void CommandCallbacks::onWrite(BLECharacteristic* pChar) {
  std::string val = pChar->getValue();
  if (BLEOtaHandler::instance && BLEOtaHandler::instance->commandHandler) {
    BLEOtaHandler::instance->commandHandler->handleCommand(val);
  }
}
