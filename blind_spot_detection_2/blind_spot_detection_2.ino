#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tflI2C;

int16_t Dist1;  // distance in centimeters
int16_t Dist2;  // distance in centimeters
int16_t Dist3;  // distance in centimeters

bool leftWarning, rightWarning, middleWarning;
bool leftWarning2, rightWarning2, middleWarning2;

uint8_t LeftLidar = 0x10;    // address of lidar 1 left
uint8_t MiddleLidar = 0x11;  // address of lidar 2 middle
uint8_t RightLidar = 0x12;   // address of lidar 3 right

const int rL = 9;  // Pin for the Red LED on the left
const int gL = 8;  // Pin for the Green LED on the left
const int bL = 7;  // Pin for the Blue LED on the left
const int rR = 4;  // Pin for the Red LED on the right
const int gR = 3;  // Pin for the Green LED on the right
const int bR = 2;  // Pin for the Blue LED on the right

// Function to get LIDAR distances
void getDistance() {
  tflI2C.getData(Dist1, LeftLidar);
  tflI2C.getData(Dist2, RightLidar);
  tflI2C.getData(Dist3, MiddleLidar);
}

// Function to calculate warnings based on distances
void calculateWarning(int16_t dist1, int16_t dist2, int16_t dist3, bool& leftWarning, bool& rightWarning, bool& middleWarning,
                      bool& leftWarning2, bool& rightWarning2, bool& middleWarning2) {
  leftWarning = dist1 < 50;    // Example threshold, adjust as needed
  rightWarning = dist2 < 50;   // Example threshold, adjust as needed
  middleWarning = dist3 < 50;  // Example threshold, adjust as needed

  leftWarning2 = dist1 >= 50 && dist1 <= 100;
  rightWarning2 = dist2 >= 50 && dist2 <= 100;
  middleWarning2 = dist3 >= 50 && dist3 <= 100;
}
void setColorLeft(int R, int G, int B) {
  analogWrite(rL, R);
  analogWrite(gL, G);
  analogWrite(bL, B);
}

void setColorRight(int R, int G, int B) {
  analogWrite(rR, R);
  analogWrite(gR, G);
  analogWrite(bR, B);
}
// Function to trigger LED warning
void ledWarning(bool leftWarning, bool rightWarning, bool middleWarning,
                bool leftWarning2, bool rightWarning2, bool middleWarning2) {
  if (leftWarning || middleWarning) {
    setColorLeft(255, 0, 0);
  } else if (leftWarning2 || middleWarning2) {
    setColorLeft(255, 255, 0);
  } else {
    setColorLeft(0, 0, 0);
  }

  if (rightWarning || middleWarning) {
    setColorRight(255, 0, 0);
  } else if (rightWarning2 || middleWarning2) {
    setColorRight(255, 255, 0);
  } else {
    setColorRight(0, 0, 0);
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
  Wire.begin();          // Initialize Wire library
  pinMode(rL, OUTPUT);
  pinMode(gL, OUTPUT);
  pinMode(bL, OUTPUT);
  pinMode(rR, OUTPUT);
  pinMode(gR, OUTPUT);
  pinMode(bR, OUTPUT);
}

void loop() {
  getDistance();
  printDistances();                                                                                                              // Print distances to Serial Monitor
  calculateWarning(Dist1, Dist2, Dist3, leftWarning, rightWarning, middleWarning, leftWarning2, rightWarning2, middleWarning2);  // Calculate warnings
  ledWarning(leftWarning, rightWarning, middleWarning, leftWarning2, rightWarning2, middleWarning2);                             // Call LED warning function
}