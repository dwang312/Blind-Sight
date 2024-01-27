#include <Wire.h>

#define SLAVE_ADDRESS_1 0x10 // I2C address for the first sensor
#define SLAVE_ADDRESS_2 0x10 // I2C address for the second sensor (modify if different)
#define COMMAND 0x00         // Command
#define DATA_LENGTH 9        // Data length

#define SDA_1 21 // Green SDA for the first sensor
#define SCL_1 22 // White SCL for the first sensor

#define SDA_2 19 // Green SDA for the second sensor
#define SCL_2 20 // White SCL for the second sensor

unsigned char buf1[] = {0x5A, 0x05, 0x00, 0x01, 0x60};
unsigned char buf2[] = {0x5A, 0x05, 0x00, 0x01, 0x60}; // Replace with the command for the second sensor

void setup()
{
  Wire.begin(SDA_1, SCL_1);
  Wire1.begin(SDA_2, SCL_2);
  Serial.begin(115200);
}

void readSensorData(int sensorAddress, unsigned char *commandBuffer, const char *sensorName, TwoWire &wireInstance)
{
  wireInstance.beginTransmission(sensorAddress);
  wireInstance.write(commandBuffer, 5);
  wireInstance.endTransmission();
  wireInstance.requestFrom(sensorAddress, DATA_LENGTH);

  uint8_t data[DATA_LENGTH] = {0};
  int index = 0;
  while (wireInstance.available() > 0 && index < DATA_LENGTH)
  {
    data[index++] = wireInstance.read();
  }

  if (index == DATA_LENGTH)
  {
    uint16_t distance = data[2] + data[3] * 256;
    uint16_t strength = data[4] + data[5] * 256;
    int16_t temperature = data[6] + data[7] * 256;

    Serial.print(sensorName);
    Serial.print(" - Distance: ");
    Serial.print(distance);
    Serial.print(" cm, Strength: ");
    Serial.print(strength);
    Serial.print(", Temperature: ");
    Serial.println(temperature / 8.0 - 256.0);
  }
}

void loop()
{
  readSensorData(SLAVE_ADDRESS_1, buf1, "Sensor 1", Wire);
  delay(1000);

  readSensorData(SLAVE_ADDRESS_2, buf2, "Sensor 2", Wire1);
  delay(1000);
}
