#ifndef BLE_OTA_HANDLER_H
#define BLE_OTA_HANDLER_H

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Update.h>
#include "CommandHandler.h"


class BLEOtaHandler {
  public:
    void begin(const char* deviceName);
    void setCommandHandler(CommandHandler* handler);
    static BLEOtaHandler* instance;
    CommandHandler* commandHandler = nullptr;


  private:
    bool otaInProgress = false;
    uint32_t otaFileSize = 0;
    uint32_t otaReceived = 0;
    


    

    class MyServerCallbacks : public BLEServerCallbacks {
      void onConnect(BLEServer* pServer);
      void onDisconnect(BLEServer* pServer);
    };

    class MyCallbacks : public BLECharacteristicCallbacks {
      void onWrite(BLECharacteristic *pCharacteristic);
    };

    void onClientConnect();
    void onClientDisconnect();
    void handleWrite(BLECharacteristic* pCharacteristic);

    // Friend classes to access private members
    friend class MyServerCallbacks;
    friend class MyCallbacks;
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  public:
    void onWrite(BLECharacteristic* pChar) override;
};


#endif
