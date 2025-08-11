#pragma once

#include <queue>
#include <string>

namespace mtrn3100 {

class CommandParser {
public: 
  CommandParser(std::string commandString) {
    std::stringstream ss(commandString);
    std::string word;
    while (!ss.eof()) {
        getline(ss, word, '|');
        commands.push(word);
    }
  }

  // For debugging
  std::string getCommand() {
    return commands.front();
  }

  char getMoveType() {
    if (commands.empty()) return '\0';
    return commands.empty() ? '\0' : commands.front()[0];
  }

  float getMoveValue() {
    if (commands.empty()) return 0;

    float number;

    if (std::stringstream(commands.front()) >> number) { 
      return number;
    }

    return 0;
  }

  void next() {
    queue.pop();
  }

  bool isEmpty() {
    return commands.empty();
  }

private: 
  std::queue<std::string> commands;
}

}

