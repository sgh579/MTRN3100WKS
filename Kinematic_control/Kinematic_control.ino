#include "DualEncoder.hpp"
#include "EncoderOdometry.hpp"
#include <SPI.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <string.h>
#include <stdio.h>



#define EN_1_A 2 //These are the pins for the PCB encoder
#define EN_1_B 7 //These are the pins for the PCB encoder
#define EN_2_A 3 //These are the pins for the PCB encoder
#define EN_2_B 8 //These are the pins for the PCB encoder

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1
#define SCREEN_ADDRESS 0x3C

mtrn3100::DualEncoder encoder(EN_1_A, EN_1_B,EN_2_A, EN_2_B);
mtrn3100::EncoderOdometry encoder_odometry(15.5, 82); //TASK1 TODO: IDENTIFY THE WHEEL RADIUS AND AXLE LENGTH

/*************** SCREEN *******************/

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

MPU6050 mpu(Wire);

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

    // screen
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        for(;;); // Don't proceed, loop forever
    }
    Serial.println(F("SSD1306 allocation completed"));

    display.display();
    delay(2000); // Pause for 2 seconds

    // Clear the buffer
    display.clearDisplay();

}

void loop() {

    delay(5);

    encoder_odometry.update(encoder.getLeftRotation(),encoder.getRightRotation());
    mpu.update();

    Serial.println(mpu.getAngleZ());

    if (loop_counter % 10 == 0) {
        Serial.print("[INFO]: Loop count");
        Serial.print(loop_counter);
        Serial.println();
    }



    loop_counter++;

    if (loop_counter > 30000) {
        Serial.println("[INFO]: Loop count exceeded 30000, resetting to 0.");
        loop_counter = 0;
    }

}

