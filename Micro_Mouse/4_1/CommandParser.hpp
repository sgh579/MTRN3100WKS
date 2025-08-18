#pragma once
#include <Arduino.h>

namespace mtrn3100 {

class CommandParser {
public:
  CommandParser(const char* commandString) : cmdString(commandString), currentPos(0) {
    findNextCommand(); // Parse first command
  }

  char getMoveType() {
    return currentMoveType;
  }

  float getMoveValue() {
    return currentMoveValue;
  }

  void next() {
    if (!isEmpty()) {
      findNextCommand();
    }
  }

  bool isEmpty() {
    return currentMoveType == '\0';
  }

private:
  const char* cmdString;        // Pointer to original string (no copy)
  int currentPos;               // Current position in string
  char currentMoveType = '\0';  // Current command type
  float currentMoveValue = 0.0f; // Current command value

  void findNextCommand() {
    currentMoveType = '\0';
    currentMoveValue = 0.0f;

    // Skip any leading delimiters
    while (cmdString[currentPos] == '|' && cmdString[currentPos] != '\0') {
      currentPos++;
    }

    // Check if we've reached the end
    if (cmdString[currentPos] == '\0') {
      return;
    }

    // Get move type (first character)
    currentMoveType = cmdString[currentPos];
    currentPos++;

    // Parse the numeric value
    int startPos = currentPos;
    
    // Find end of number (next '|' or end of string)
    while (cmdString[currentPos] != '|' && cmdString[currentPos] != '\0') {
      currentPos++;
    }

    // Convert substring to float
    if (currentPos > startPos) {
      currentMoveValue = parseFloat(startPos, currentPos);
    }
  }

  // Custom float parser to avoid creating temporary strings
  float parseFloat(int start, int end) {
    float result = 0.0f;
    float sign = 1.0f;
    float decimal = 0.1f;
    bool inDecimal = false;
    
    int pos = start;
    
    // Handle negative sign
    if (pos < end && cmdString[pos] == '-') {
      sign = -1.0f;
      pos++;
    } else if (pos < end && cmdString[pos] == '+') {
      pos++;
    }
    
    // Parse digits
    while (pos < end) {
      char c = cmdString[pos];
      
      if (c >= '0' && c <= '9') {
        if (!inDecimal) {
          result = result * 10.0f + (c - '0');
        } else {
          result += (c - '0') * decimal;
          decimal *= 0.1f;
        }
      } else if (c == '.' && !inDecimal) {
        inDecimal = true;
      } else {
        break; // Invalid character, stop parsing
      }
      pos++;
    }
    
    return result * sign;
  }
};

} // namespace mtrn3100