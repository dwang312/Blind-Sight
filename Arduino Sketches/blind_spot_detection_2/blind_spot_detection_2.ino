#include <Arduino.h>
#include <TFLI2C.h>

TFLI2C tflI2C;

int16_t Dist1;  // distance in centimeters
int16_t Dist2;  // distance in centimeters
int16_t Dist3;  // distance in centimeters

bool leftAZ, rightAZ, middleAZ;  // alert zone less then 4 meters
bool leftWZ, rightWZ, middleWZ;  // warning zone 4 to 8 meters

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
  tflI2C.getData(Dist1, static_cast<uint16_t>(LeftLidar));
  tflI2C.getData(Dist2, static_cast<uint16_t>(RightLidar));
  tflI2C.getData(Dist3, static_cast<uint16_t>(MiddleLidar));
}

// Function to calculate warnings based on distances
void calculateWarning(int16_t dist1, int16_t dist2, int16_t dist3, bool& leftAZ, bool& rightAZ, bool& middleAZ,
                      bool& leftWZ, bool& rightWZ, bool& middleWZ) {
  leftAZ = dist1 >= 10 && dist1 < 400;    // Example threshold, adjust as needed
  rightAZ = dist2 >= 10 && dist2 < 400;   // Example threshold, adjust as needed
  middleAZ = dist3 >= 10 && dist3 < 400;  // Example threshold, adjust as needed

  leftWZ = dist1 >= 400 && dist1 <= 800;
  rightWZ = dist2 >= 400 && dist2 <= 800;
  middleWZ = dist3 >= 400 && dist3 <= 800;
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
void ledWarning(bool leftAZ, bool rightAZ, bool middleAZ,
                bool leftWZ, bool rightWZ, bool middleWZ) {
  if (middleAZ || leftAZ) {
    setColorLeft(255, 0, 0);  // Red for close objects
  } else if (middleWZ || leftWZ) {
    setColorLeft(255, 255, 0);  // Yellow for intermediate range
  } else {
    setColorLeft(0, 0, 0);  // Turn off LEDs
  }

  if (middleAZ || rightAZ) {
    setColorRight(255, 0, 0);  // Red for close objects
  } else if (middleWZ || rightWZ) {
    setColorRight(255, 255, 0);  // Yellow for intermediate range
  } else {
    setColorRight(0, 0, 0);  // Turn off LEDs
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
  pinMode(rL, OUTPUT);
  pinMode(gL, OUTPUT);
  pinMode(bL, OUTPUT);
  pinMode(rR, OUTPUT);
  pinMode(gR, OUTPUT);
  pinMode(bR, OUTPUT);
}

void loop() {
  getDistance();  // Get LIDAR data
  // printDistances();                                                                                                         // Print distances to Serial Monitor (commented out for this example)
  calculateWarning(Dist1, Dist2, Dist3, leftAZ, rightAZ, middleAZ, leftWZ, rightWZ, middleWZ);  // Calculate warnings
  ledWarning(leftAZ, rightAZ, middleAZ, leftWZ, rightWZ, middleWZ);                             // Call LED warning function
}
