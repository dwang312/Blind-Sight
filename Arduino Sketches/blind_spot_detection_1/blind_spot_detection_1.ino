#include <Arduino.h>
#include <TFLI2C.h>      // TFLuna-I2C Library v.0.1.1

TFLI2C tflI2C;

int16_t Dist1;    // distance in centimeters
int16_t Dist2;    // distance in centimeters
int16_t Dist3;    // distance in centimeters

bool warning1, warning2, warning3;

uint8_t  MiddleLidar = 0x10;  // address of lidar 1 middle
uint8_t  LeftLidar = 0x11;  // address of lidar 2 left
uint8_t  RightLidar = 0x12;  // address of lidar 3 right

const int ledPin1 = 6; // Pin for the first LED
const int ledPin2 = 7; // Pin for the second LED

// Function to get LIDAR distances
void getDistance() {
    tflI2C.getData(Dist1, LeftLidar);
    tflI2C.getData(Dist2, RightLidar);
    tflI2C.getData(Dist3, MiddleLidar);
}

// Function to calculate warnings based on distances
void calculateWarning(int16_t dist1, int16_t dist2, int16_t dist3, bool& warning1, bool& warning2, bool& warning3) {
    warning1 = dist1 < 50; // Example threshold, adjust as needed
    warning2 = dist2 < 50; // Example threshold, adjust as needed
    warning3 = dist3 < 50; // Example threshold, adjust as needed
}

// Function to send Bluetooth warning
void bluetoothWarning(bool warning1, bool warning2) {
    // Implement Bluetooth warning signal
}

// Function to trigger LED warning
void ledWarning(bool warning1, bool warning2, bool warning3) {
    digitalWrite(ledPin1, warning1 ? HIGH : LOW); // Turn on/off first LED based on warning1
    digitalWrite(ledPin2, warning2 ? HIGH : LOW); // Turn on/off second LED based on warning2
    if (warning3) {
        digitalWrite(ledPin1, HIGH); // Turn on first LED
        digitalWrite(ledPin2, HIGH); // Turn on second LED
    }
}


// Function to print distances to Serial Monitor
void printDistances() {
    Serial.print("Left Distance: ");
    Serial.print(Dist1);
    Serial.print(" cm, Right Distance: ");
    Serial.print(Dist2);
    Serial.print(" cm Middle Distance: ");
    Serial.print(Dist3);
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
    calculateWarning(Dist1, Dist2, Dist3, warning1, warning2, warning3); // Calculate warnings
    ledWarning(warning1, warning2, warning3); // Call LED warning function
    //bluetoothWarning(warnings.first, warnings.second);
    delay(100);
}
