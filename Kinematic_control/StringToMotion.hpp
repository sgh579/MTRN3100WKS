//
// Created by Naveen Thayyil on 13/7/2025.
//
// set up as a main file

#pragma once

#include <Arduino.h>


String commandString = "";

void setup() {
    Serial.begin(9600);
    while (!Serial) {}
    Serial.println("Enter movement string:");
}

void loop() {
    void loop() {
        if (Serial.available() > 5) {        // # of char to be read fron input - made it > 5 as
            //    we will recieve 8 movements
            commandString = Serial.readStringUntil('\n');
            commandString.trim();  // Remove trailing newline/space
            Serial.print("Received command: ");
            Serial.println(commandString);

            executeCommands(commandString);
        }
    }
}

void executeCommands(String commands) {
    for (int i = 0; i < commands.length(); i++) {
        char c = commands.charAt(i);
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