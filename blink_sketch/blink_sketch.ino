const int ledPin1 = 6; // Pin for the first LED
const int ledPin2 = 7; // Pin for the second LED

void setup() {
  pinMode(ledPin1, OUTPUT); // Set the LED pin as an output
  pinMode(ledPin2, OUTPUT); // Set the LED pin as an output
}

void loop() {
  digitalWrite(ledPin1, HIGH); // Turn on the first LED
  digitalWrite(ledPin2, LOW); // Turn off the second LED
  delay(500); // Wait for 500 milliseconds

  digitalWrite(ledPin1, LOW); // Turn off the first LED
  digitalWrite(ledPin2, HIGH); // Turn on the second LED
  delay(500); // Wait for 500 milliseconds
}
