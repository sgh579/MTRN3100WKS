#pragma once

#include <Arduino.h>

namespace mtrn3100 {
class EncoderOdometry {
public:
    EncoderOdometry(float radius, float wheelBase) : x(0), y(0), h(0), R(radius), B(wheelBase), lastLPos(0), lastRPos(0) {}

    // COMPLETE THIS FUNCTION
    void update(float leftValue,float rightValue) {

        // Calculate the change in radians since the last update.
        float delta_left_radians = leftValue-lastLPos; // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEYARE NOT THE WRONG DIRECTION 
        float delta_right_radians = -(rightValue-lastRPos); // MAKE SURE THE ENCODERS COUNT UP CORRECTLY IE. THEY ARE NOT THE WRONG DIRECTION 


        // Calculate the foward kinematics
        float delta_s = R * 0.5 * (delta_left_radians + delta_right_radians);
        float delta_theta = (R / B) * (-delta_left_radians + delta_right_radians);
        x += delta_s * cos(h);    
        y += delta_s * sin(h);    
        h += delta_theta;
        lastLPos = leftValue;
        lastRPos = rightValue;
    }

    float getX() const { return x; }
    float getY() const { return y; }
    float getH() const { return h; }

private:
    float x, y, h;
    const float R, B;
    float lastLPos, lastRPos;
};

}
