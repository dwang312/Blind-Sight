#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tflI2C;

int16_t Dist1;    // distance in centimeters
int16_t Dist2;    // distance in centimeters
int16_t Dist3;    // distance in centimeters

bool leftWarning, rightWarning, middleWarning;
bool leftWarning2, rightWarning2, middleWarning2;

uint8_t MiddleLidar = 0x10;  // address of lidar 1 middle
uint8_t LeftLidar = 0x11;    // address of lidar 2 left
uint8_t RightLidar = 0x12;   // address of lidar 3 right

const int rL = 9; // Pin for the Red LED on the left
const int gL = 8; // Pin for the Green LED on the left
const int bL = 7; // Pin for the Blue LED on the left
const int rR = 4; // Pin for the Red LED on the right
const int gR = 3; // Pin for the Green LED on the right
const int bR = 2; // Pin for the Blue LED on the right

unsigned long previousMillis = 0;
const long interval = 1000; // Flashing interval in milliseconds

// Function to get LIDAR distances
void getDistance() {
    tflI2C.getData(Dist1, LeftLidar);
    tflI2C.getData(Dist2, RightLidar);
    tflI2C.getData(Dist3, MiddleLidar);
}

// Function to calculate warnings based on distances
void calculateWarning(int16_t dist1, int16_t dist2, int16_t dist3, bool& leftWarning, bool& rightWarning, bool& middleWarning,
                      bool& leftWarning2, bool& rightWarning2, bool& middleWarning2) {
    leftWarning = dist1 < 50; // Example threshold, adjust as needed
    rightWarning = dist2 < 50; // Example threshold, adjust as needed
    middleWarning = dist3 < 50; // Example threshold, adjust as needed

    leftWarning2 = dist1 >= 50 && dist1 <= 100;
    rightWarning2 = dist2 >= 50 && dist2 <= 100;
    middleWarning2 = dist3 >= 50 && dist3 <= 100;
}

// Function to trigger LED warning
void ledWarning(bool leftWarning, bool rightWarning, bool middleWarning,
                bool leftWarning2, bool rightWarning2, bool middleWarning2) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;

        if (leftWarning) {
            digitalWrite(rL, HIGH);
            digitalWrite(gL, LOW);
            digitalWrite(bL, LOW);
            delay(100);
            digitalWrite(rL, LOW);
            delay(100);
        } else if (leftWarning2) {
            digitalWrite(rL, HIGH);
            digitalWrite(gL, HIGH);
            digitalWrite(bL, LOW);
            delay(100);
            digitalWrite(rL, LOW);
            digitalWrite(gL, LOW);
            delay(100);
        } else {
            digitalWrite(rL, LOW);
            digitalWrite(gL, LOW);
            digitalWrite(bL, LOW);
        }

        if (rightWarning) {
            digitalWrite(rR, HIGH);
            digitalWrite(gR, LOW);
            digitalWrite(bR, LOW);
            delay(100);
            digitalWrite(rR, LOW);
            delay(100);
        } else if (rightWarning2) {
            digitalWrite(rR, HIGH);
            digitalWrite(gR, HIGH);
            digitalWrite(bR, LOW);
            delay(100);
            digitalWrite(rR, LOW);
            digitalWrite(gR, LOW);
            delay(100);
        } else {
            digitalWrite(rR, LOW);
            digitalWrite(gR, LOW);
            digitalWrite(bR, LOW);
        }

        if (middleWarning) {
            digitalWrite(rR, HIGH);
            digitalWrite(gR, LOW);
            digitalWrite(bR, LOW);
            digitalWrite(rL, HIGH);
            digitalWrite(gL, LOW);
            digitalWrite(bL, LOW);
            delay(100);
            digitalWrite(rR, LOW);
            digitalWrite(rL, LOW);
            delay(100);
        } else if (middleWarning2) {
            digitalWrite(rR, HIGH);
            digitalWrite(gR, HIGH);
            digitalWrite(bR, LOW);
            digitalWrite(rL, HIGH);
            digitalWrite(gL, HIGH);
            digitalWrite(bL, LOW);
            delay(100);
            digitalWrite(rR, LOW);
            digitalWrite(gR, LOW);
            digitalWrite(rL, LOW);
            digitalWrite(gL, LOW);

            delay(100);
        } else {
            digitalWrite(rR, LOW);
            digitalWrite(gR, LOW);
            digitalWrite(bR, LOW);
            digitalWrite(rL, LOW);
            digitalWrite(gL, LOW);
            digitalWrite(bL, LOW);
        }
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
    Wire.begin();           // Initialize Wire library
    pinMode(rL, OUTPUT);
    pinMode(gL, OUTPUT);
    pinMode(bL, OUTPUT);
    pinMode(rR, OUTPUT);
    pinMode(gR, OUTPUT);
    pinMode(bR, OUTPUT);
}

void loop() {
    getDistance();
    printDistances(); // Print distances to Serial Monitor
    calculateWarning(Dist1, Dist2, Dist3, leftWarning, rightWarning, middleWarning,
                      leftWarning2, rightWarning2, middleWarning2); // Calculate warnings
    ledWarning(leftWarning, rightWarning, middleWarning,
                leftWarning2, rightWarning2, middleWarning2); // Call LED warning function
}
