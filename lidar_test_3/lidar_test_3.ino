#include <Arduino.h>
#include <Wire.h>        // Instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1

TFLI2C tflI2C;

int16_t  Dist1;    // distance in centimeters
int16_t  Dist2;    // distance in centimeters
int8_t  sensor1 = 0x11;  // address
int8_t  sensor2 = 0x12;  // address
int8_t nDevice = 2;

void getDistance(const char *name, int16_t dist, int8_t addr)
{
    if(tflI2C.getData(dist, addr)){
        Serial.print(name);
        Serial.print(": ");
        Serial.print(dist);
        Serial.print(" cm \n");
    }
}

void setup(){
    Serial.begin(115200);  // Initialize serial port
    Wire.begin();           // Initialize Wire library
}

void loop(){
    getDistance("lidar1", Dist1, sensor1);
    delay(1000);
    getDistance("lidar2", Dist2, sensor2);
    delay(1000);
}
