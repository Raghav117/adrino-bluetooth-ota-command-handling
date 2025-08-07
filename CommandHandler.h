#ifndef COMMAND_HANDLER_H
#define COMMAND_HANDLER_H

#include <Arduino.h>

class CommandHandler {
  public:
    void begin();
    void handleCommand(const std::string& command);
    String getLastCommand();
    bool hasNewCommand();

  private:
    String lastCommand = "";
    bool newCommand = false;
};

#endif
