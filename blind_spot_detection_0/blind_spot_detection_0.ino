#include <Arduino.h>
#include <Wire.h>        // Instantiate the Wire library
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1

TFLI2C tflI2C;

int16_t lidarDist1;    // distance in centimeters
int16_t lidarDist2;    // distance in centimeters
int8_t  sensor1 = 0x11;  // address
int8_t  sensor2 = 0x12;  // address

const int ledPin1 = 6; // Pin for the first LED
const int ledPin2 = 7; // Pin for the second LED

// Function to get LIDAR distances
void getLIDARDistance() {
    tflI2C.getData(lidarDist1, sensor1);
    tflI2C.getData(lidarDist2, sensor2);
}

// Function to calculate warnings
std::pair<bool, bool> calculateWarning(int16_t dist1, int16_t dist2) {
    bool warning1 = dist1 < 50; // Example threshold, adjust as needed
    bool warning2 = dist2 < 50; // Example threshold, adjust as needed
    return std::make_pair(warning1, warning2);
}

// Function to send Bluetooth warning
void bluetoothWarning(bool warning1, bool warning2) {
    // Implement Bluetooth warning signal
}

// Function to trigger LED warning
void ledWarning(bool warning1, bool warning2) {
    digitalWrite(ledPin1, warning1 ? HIGH : LOW); // Turn on/off first LED based on warning1
    digitalWrite(ledPin2, warning2 ? HIGH : LOW); // Turn on/off second LED based on warning2
}

// Function to print distances to Serial Monitor
void printDistances() {
    Serial.print("Distance 1: ");
    Serial.print(lidarDist1);
    Serial.print(" cm, Distance 2: ");
    Serial.print(lidarDist2);
    Serial.println(" cm");
}

void setup() {
    Serial.begin(115200);  // Initialize serial port
    Wire.begin();           // Initialize Wire library
    pinMode(ledPin1, OUTPUT); // Set the LED pin as an output
    pinMode(ledPin2, OUTPUT); // Set the LED pin as an output
}

void loop() {
    getLIDARDistance();
    printDistances(); // Print distances to Serial Monitor
    std::pair<bool, bool> warnings = calculateWarning(lidarDist1, lidarDist2);
    bluetoothWarning(warnings.first, warnings.second);
    ledWarning(warnings.first, warnings.second);
    delay(10);
}
