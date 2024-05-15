#include <Arduino.h>
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1

TFLI2C tflI2C;

int16_t Dist1;    // distance in centimeters
int16_t Dist2;    // distance in centimeters


uint8_t  lidar1 = 0x11;  // address of lidar 1
uint8_t  lidar2 = 0x12;  // address of lidar 2 


const int ledPin1 = 6; // Pin for the first LED
const int ledPin2 = 7; // Pin for the second LED

// Function to get LIDAR distances
void getDistance() {
    tflI2C.getData(Dist1, lidar1);
    tflI2C.getData(Dist2, lidar2);
}

// Function to calculate warnings
std::pair<bool, bool> calculateWarning(int16_t dist1, int16_t dist2) {
    bool warning1 = dist1 < 150; // Example threshold, adjust as needed
    bool warning2 = dist2 < 150; // Example threshold, adjust as needed
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
    Serial.print(Dist1);
    Serial.print(" cm, Distance 2: ");
    Serial.print(Dist2);
    Serial.println(" cm");
}

void setup() {
    Serial.begin(115200);  // Initialize serial port
    pinMode(ledPin1, OUTPUT); // Set the LED pin as an output
    pinMode(ledPin2, OUTPUT); // Set the LED pin as an output
}

void loop() {
    getDistance();
    printDistances(); // Print distances to Serial Monitor
    std::pair<bool, bool> warnings = calculateWarning(Dist1, Dist2);
    //bluetoothWarning(warnings.first, warnings.second);
    ledWarning(warnings.first, warnings.second);
    delay(10);
}
