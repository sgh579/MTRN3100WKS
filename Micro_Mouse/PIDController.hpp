#pragma once

#include <math.h>

namespace mtrn3100 {

class PIDController {
public:
    PIDController(float kp, float ki, float kd) : kp(kp), ki(ki), kd(kd) {}

    // Compute the output signal required from the current/actual value.
    float compute(float input) {
        if (!turn_on_flag) return 0.0f; 
      
        curr_time = micros();
        dt = static_cast<float>(curr_time - prev_time) / 1e6;
        prev_time = curr_time;

        error = setpoint - (input - zero_ref);

        // IMPLIMENT PID CONTROLLER
        integral = integral + error * dt;
        derivative = (error - prev_error) / dt;
        output = kp*error + ki*integral + kd*derivative;

        prev_error = error;

        return output;
        
    }

    // Function used to return the last calculated error. 
    // The error is the difference between the desired position and current position. 
    void tune(float p, float i, float d) {
        kp = p;
        ki = i;
        kd = d;
    }

    float getError() {
      return error;
    }

    // This must be called before trying to achieve a setpoint.
    // The first argument becomes the new zero reference point.
    // Target is the setpoint value.
    void zeroAndSetTarget(float zero, float target) {
        zero_ref = zero;
        setpoint = target;
    }

    void setTarget(float target)
    {  
        setpoint = target;
    }

    void setZeroRef(float zero) {
        zero_ref = zero;
    }

    void disable() {
        turn_on_flag = false;
    }

    void enable() {
        turn_on_flag = true;
    }

    void reset(){
        prev_time = micros();
        curr_time = prev_time;
        prev_error = 0;
        setpoint = 0;
        zero_ref = 0;
    }


public:
    uint32_t prev_time, curr_time = micros();
    float dt;

private:
    float kp, ki, kd;
    float error, derivative, integral, output;
    float prev_error = 0;
    float setpoint = 0;
    float zero_ref = 0;
    bool turn_on_flag = true;

    
};

}  // namespace mtrn3100
