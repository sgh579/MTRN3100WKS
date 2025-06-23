#include "Encoder.hpp"
#include "Motor.hpp"
#include "PIDController.hpp"

#define MOT1PWM 11 // PIN 11 is motor1's PWM pin
#define MOT1DIR 12

#define MOT2PWM 9// PIN 9 is motor2's PWM pin
#define MOT2DIR 10

// TODO: create a struct for this, avoid using 2 separate motor objects
mtrn3100::Motor motor1(MOT1PWM,MOT1DIR);
mtrn3100::Motor motor2(MOT2PWM,MOT2DIR);

#define EN1_A 2 // PIN 2 is encoder1's interupt
#define EN1_B 7

#define EN2_A  3// PIN ? is encoder2's interupt
#define EN2_B  8

mtrn3100::Encoder encoder1(EN1_A, EN1_B);
mtrn3100::Encoder encoder2(EN2_A, EN2_B);

// this implementation is a PID controller based on location
// we need a new implementation that is unversial and can be used for speed control, path following, etc.
mtrn3100::PIDController controller1(600, 1, 10); // Kp, Ki, Kd , motor1's position controller
mtrn3100::PIDController controller2(600, 1, 10); // Kp, Ki, Kd , motor2's position controller

void setup() {
  Serial.begin(9600);

  float target_motion_length = 1000; // 1000 mm, specified by task4
  float motion_length_to_rotation_scale = 1; // to be adjusted based on the motor and encoder specifications
  float target_motion_rotation_radians = target_motion_length * motion_length_to_rotation_scale * (2.0f * M_PIF / encoder1.counts_per_revolution);

  controller1.zeroAndSetTarget(encoder1.getRotation(), target_motion_rotation_radians); 
  controller2.zeroAndSetTarget(encoder2.getRotation(), -target_motion_rotation_radians); // reverse it for vehicle's motion
}

void loop() {

  int controller1_output = controller1.compute(encoder1.getRotation());
  int controller2_output = controller2.compute(encoder2.getRotation());
  
  motor1.setPWM(controller1_output); 
  motor2.setPWM(controller2_output); 

  delay(50);  // loop interval, picked by random, to be adjusted based on the application requirements

}