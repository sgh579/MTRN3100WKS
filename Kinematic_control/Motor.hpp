#pragma once

#define MAXM_SPEED 50

#include <Arduino.h>

#include "math.h"

namespace mtrn3100 {


// The motor class is a simple interface designed to assist in motor control
// You may choose to impliment additional functionality in the future such as dual motor or speed control 
class Motor {
public:
    Motor( uint8_t pwm_pin, uint8_t in2) :  pwm_pin(pwm_pin), dir_pin(in2) {
        //  Setboth pins as output
        pinMode(pwm_pin, OUTPUT);
        pinMode(dir_pin, OUTPUT);
    }


    // This function outputs the desired motor direction and the PWM signal. 
    // NOTE: a pwm signal > 255 could cause troubles as such ensure that pwm is clamped between 0 - 255.

    void setPWM(int16_t pwm) {

      // Output digital direction pin based on if input signal is positive or negative.
      // Output PWM signal between 0 - 255.
      int speed = abs(pwm);
      if (speed > MAXM_SPEED)
      {
        speed = MAXM_SPEED;
      }
      analogWrite(pwm_pin, speed); // Write the absolute value of pwm to the PWM pin
      digitalWrite(dir_pin, pwm >= 0 ? HIGH : LOW); // Set direction pin
    }

private:
    const uint8_t pwm_pin;
    const uint8_t dir_pin;
};

}  // namespace mtrn3100
