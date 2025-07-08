#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include "Wire.h"
#include <MPU6050_light.h>

MPU6050 mpu(Wire);

#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH


int loop_counter = 0;

void setup() {
    Serial.begin(115200);
    Wire.begin();

    //Set up the IMU
    byte status = mpu.begin();
    Serial.print(F("MPU6050 status: "));
    Serial.println(status);
    while(status!=0){ } // stop everything if could not connect to MPU6050
    
    Serial.println(F("Calculating offsets, do not move MPU6050"));
    delay(1000);
    mpu.calcOffsets(true,true);
    Serial.println("Done!\n");
}

void loop() {

    delay(50);

    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());

    mpu.update();



    if (loop_counter % 10 == 0) {
        Serial.print("[INFO]: Loop count");
        Serial.print(loop_counter);
        Serial.println();
    }
    loop_counter++;
    if (loop_counter > 60000) {
        Serial.println("[INFO]: Loop count exceeded 60000, resetting to 0.");
        loop_counter = 0;
    }
}