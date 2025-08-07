#include "CommandHandler.h"

void CommandHandler::begin() {
  // Initialize if needed
}

void CommandHandler::handleCommand(const std::string& command) {
  lastCommand = String(command.c_str());
  newCommand = true;
}

String CommandHandler::getLastCommand() {
  newCommand = false;
  return lastCommand;
}

bool CommandHandler::hasNewCommand() {
  return newCommand;
}
