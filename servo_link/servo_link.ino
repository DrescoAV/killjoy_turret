#include <Servo.h>

const int servoXPin = 9;  // Define the pin for the X-axis servo
const int servoYPin = 10; // Define the pin for the Y-axis servo

Servo servoX;
Servo servoY;

void setup() {
  Serial.begin(9600);  // Initialize serial communication
  servoX.attach(servoXPin);  // Attach the X-axis servo to pin 9
  servoY.attach(servoYPin);  // Attach the Y-axis servo to pin 10

  servoX.write(90);
  servoY.write(90);
}

void loop() {
  if (Serial.available()) {
    // Read the incoming serial data
    String data = Serial.readStringUntil('\n');
    int commaIndex = data.indexOf(',');
    
    if (commaIndex > 0) {
      // Extract X and Y coordinates
      int servoXAngle = data.substring(0, commaIndex).toInt();
      int servoYAngle = data.substring(commaIndex + 1).toInt();
      
      // Map angles if necessary (0-180 is already the range for servos)
      servoX.write(servoXAngle);  // Set the X-axis servo position
      servoY.write(servoYAngle);  // Set the Y-axis servo position
    }
  }
}
