#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>  // TFLuna-I2C Library v.0.1.1

TFLI2C tflI2C;

void setup() {
  Serial.begin(115200);  // Initialize Serial port
  Wire.begin();           // Initialize Wire library

  Serial.println("TFLI2C example code: 10 JUL 2021\n");

  Serial.println("Enter current I2C address in hexadecimal format (e.g., 0x29):");
  while (!Serial.available()); // Wait until user inputs data
  String input = Serial.readStringUntil('\n'); // Read entire line of input
  uint8_t currentAddr = strtol(input.c_str(), NULL, 16); // Parse hexadecimal string as integer
  Serial.print("Current address set to: 0x");
  Serial.println(currentAddr, HEX);

  Serial.println("Enter new I2C address in hexadecimal format (e.g., 0x12):");
  while (!Serial.available()); // Wait until user inputs data
  input = Serial.readStringUntil('\n'); // Read entire line of input
  uint8_t newAddr = strtol(input.c_str(), NULL, 16); // Parse hexadecimal string as integer
  Serial.print("New address set to: 0x");
  Serial.println(newAddr, HEX);

  tflI2C.Set_I2C_Addr(newAddr, currentAddr); // Library function to change address
  delay(1000);

  tflI2C.Save_Settings(newAddr); // Save changes
  delay(1000);

  Serial.println("System Reset:");
  if (tflI2C.Soft_Reset(newAddr)) {
    Serial.println("Passed");
  } else {
    tflI2C.printStatus();  // For troubleshooting, not necessary for operation
  }
}

void loop() {
  // Leave loop empty since we're not doing continuous operations in this example
}
