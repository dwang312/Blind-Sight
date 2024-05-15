#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

TFLI2C tflI2C;

int16_t leftDist;    // distance in centimeters from left LIDAR
int16_t rightDist;   // distance in centimeters from right LIDAR
int16_t middleDist;  // distance in centimeters from middle LIDAR

bool leftAZ, rightAZ, middleAZ;  // alert zone: less than 4 meters
bool leftWZ, rightWZ, middleWZ;  // warning zone: 4 to 8 meters

uint8_t LeftLidar = 0x10;    // address of left LIDAR sensor
uint8_t MiddleLidar = 0x11;  // address of middle LIDAR sensor
uint8_t RightLidar = 0x12;   // address of right LIDAR sensor

const int rL = 9;  // Pin for the Red LED on the left
const int gL = 8;  // Pin for the Green LED on the left
const int bL = 7;  // Pin for the Blue LED on the left
const int rR = 4;  // Pin for the Red LED on the right
const int gR = 3;  // Pin for the Green LED on the right
const int bR = 2;  // Pin for the Blue LED on the right

unsigned long leftBlinkInterval = 150;   // Interval for left LED blinking (default: 150 milliseconds)
unsigned long rightBlinkInterval = 150;  // Interval for right LED blinking (default: 150 milliseconds)
unsigned long lastBlinkLeft = 0;         // Timestamp of the last blink for the left LED (initially set to 0)
unsigned long lastBlinkRight = 0;        // Timestamp of the last blink for the right LED (initially set to 0)
bool leftLED = true;                     // Flag indicating whether the left LED is on (default: true)
bool rightLED = true;                    // Flag indicating whether the right LED is on (default: true)

// Function to get LIDAR distances
void getDistance() {
  tflI2C.getData(leftDist, static_cast<uint16_t>(LeftLidar));
  tflI2C.getData(rightDist, static_cast<uint16_t>(RightLidar));
  tflI2C.getData(middleDist, static_cast<uint16_t>(MiddleLidar));
}

// Function to calculate warnings based on distances
void calculateWarning(int16_t leftDist, int16_t rightDist, int16_t middleDist, bool& leftAZ, bool& rightAZ, bool& middleAZ,
                      bool& leftWZ, bool& rightWZ, bool& middleWZ) {
  // Set alert zone flags based on distance thresholds
  leftAZ = leftDist >= 10 && leftDist < 40;
  rightAZ = rightDist >= 10 && rightDist < 40;
  middleAZ = middleDist >= 10 && middleDist < 40;

  // Set warning zone flags based on distance thresholds
  leftWZ = leftDist >= 40 && leftDist <= 80;
  rightWZ = rightDist >= 40 && rightDist <= 80;
  middleWZ = middleDist >= 40 && middleDist <= 80;

  // Adjust blink intervals based on distances
  if (middleDist < leftDist || middleDist < rightDist) {
    leftBlinkInterval = map(middleDist, 10, 80, 100, 200);
    rightBlinkInterval = map(middleDist, 10, 80, 100, 200);
  } else {
    leftBlinkInterval = map(leftDist, 10, 80, 100, 200);
    rightBlinkInterval = map(rightDist, 10, 80, 100, 200);
  }
}

// Function to set RGB color for the left LED
void setColorLeft(int R, int G, int B) {
  analogWrite(rL, R);
  analogWrite(gL, G);
  analogWrite(bL, B);
}

// Function to set RGB color for the right LED
void setColorRight(int R, int G, int B) {
  analogWrite(rR, R);
  analogWrite(gR, G);
  analogWrite(bR, B);
}

// Function to trigger LED warning based on zones
void ledWarning(bool leftAZ, bool rightAZ, bool middleAZ, bool leftWZ, bool rightWZ, bool middleWZ) {
  // Check and update left LED based on zones and blink intervals
  if (millis() - lastBlinkLeft >= leftBlinkInterval) {
    if ((middleAZ || leftAZ) && leftLED) {
      setColorLeft(255, 0, 0);  // Red for close objects
      leftLED = false;
    } else if ((middleWZ || leftWZ) && leftLED) {
      setColorLeft(255, 255, 0);  // Yellow for intermediate range
      leftLED = false;
    } else {
      setColorLeft(0, 0, 0);  // Turn off LEDs
      leftLED = true;
    }
    lastBlinkLeft = millis();
  }
  // Check and update right LED based on zones and blink intervals
  if (millis() - lastBlinkRight >= rightBlinkInterval) {
    if ((middleAZ || rightAZ) && rightLED) {
      setColorRight(255, 0, 0);  // Red for close objects
      rightLED = false;
    } else if ((middleWZ || rightWZ) && rightLED) {
      setColorRight(255, 255, 0);  // Yellow for intermediate range
      rightLED = false;
    } else {
      setColorRight(0, 0, 0);  // Turn off LEDs
      rightLED = true;
    }
    lastBlinkRight = millis();
  }
}

// Function to print distances to Serial Monitor
void printDistances() {
  Serial.print("Left Distance: ");
  Serial.print(leftDist);
  Serial.print(" cm, Right Distance: ");
  Serial.print(rightDist);
  Serial.print(" cm Middle Distance: ");
  Serial.print(middleDist);
  Serial.println(" cm");
}

void setup() {
  Serial.begin(115200);  // Initialize serial port
  Wire.begin();          // Initialize Wire library
  pinMode(rL, OUTPUT);   // Set LED pins as outputs
  pinMode(gL, OUTPUT);
  pinMode(bL, OUTPUT);
  pinMode(rR, OUTPUT);
  pinMode(gR, OUTPUT);
  pinMode(bR, OUTPUT);
}

void loop() {
  getDistance();  // Get LIDAR data
  // printDistances();                       // Print distances to Serial Monitor (commented out for this example)
  calculateWarning(leftDist, rightDist, middleDist, leftAZ, rightAZ, middleAZ, leftWZ, rightWZ, middleWZ);  // Calculate warnings
  ledWarning(leftAZ, rightAZ, middleAZ, leftWZ, rightWZ, middleWZ);                                         // Call LED warning function
}
