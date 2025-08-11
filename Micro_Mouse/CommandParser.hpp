#pragma once
#include <Arduino.h>
#include <string.h> // strtok, strncpy
#include <stdlib.h> // atof

namespace mtrn3100 {

class CommandParser {
public:
  CommandParser(const char* commandString) {
    // Copy to buffer (for strtok)
    strncpy(buffer, commandString, sizeof(buffer) - 1);
    buffer[sizeof(buffer) - 1] = '\0';

    // Tokenize with '|' delimiter
    char* token = strtok(buffer, "|");
    while (token != NULL && cmdCount < MAX_COMMANDS) {
      moveTypes[cmdCount] = token[0];              // first char is move type
      moveValues[cmdCount] = atof(token + 1);      // rest is the number
      cmdCount++;
      token = strtok(NULL, "|");
    }
  }

  char getMoveType() {
    if (!isEmpty()) {
      return moveTypes[cmdIndex];
    }
    return '\0';
  }

  float getMoveValue() {
    if (!isEmpty()) {
      return moveValues[cmdIndex];
    }
    return 0.0f;
  }

  void next() {
    if (!isEmpty()) {
      cmdIndex++;
    }
  }

  bool isEmpty() {
    return cmdIndex >= cmdCount;
  }

private:
  static const int MAX_COMMANDS = 10; // TODO: Adjust
  char buffer[200];                   // holds raw input string
  char moveTypes[MAX_COMMANDS];       // stores the letter for each command
  float moveValues[MAX_COMMANDS];     // stores the numeric value for each command
  int cmdCount = 0;
  int cmdIndex = 0;
};

} // namespace mtrn3100
