//
// Created by Naveen Thayyil on 13/7/2025.
//
// set up as a main file

#pragma once

#include <Arduino.h>


char commandString[9] = ""; // 9 accounting for error

void setup() {
    Serial.begin(9600);
    //    while (!Serial) {}
}

void loop() {
    if (commandString[0] != '\0') {
        Serial.println(commandString);
        executeCommands(commandString);
    } else {
      Serial.println("Completing other commands");
    }
}

void executeCommands(char* commands) {
    for (int i = 0; commands[i] != '\0'; i++) {
        char c = commands[i];
        switch (c) {
            case 'f':
                moveForwardOneCell();
            break;
            case 'l':
                turnLeft90();
            break;
            case 'r':
                turnRight90();
            break;
            default:
                Serial.print("Invalid command: ");
            Serial.println(c);
            break;
        }
    }
}

// movement functions
void moveForwardOneCell() {
    float cellLength = 180.0f; // mm
    driveStraight(cellLength); // drive function
}
void turnLeft90() {
    rotateInPlace(-90); // Negative for CCW
}

void turnRight90() {
    rotateInPlace(90); // Positive for CW
}

bool motionComplete(float leftTarget, float rightTarget, float tolerance = 0.01) { // unsure on how to exactly implement it into the main function
    float leftError = abs(leftTarget - encoder.getLeftRotation());
    float rightError = abs(rightTarget - encoder.getRightRotation());

    return leftError < tolerance && rightError < tolerance;
}