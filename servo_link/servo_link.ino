#include <Servo.h>

const int servoXPin = 9;  // Define the pin for the X-axis servo
const int servoYPin = 10; // Define the pin for the Y-axis servo
const int joyXPin = A0;   // Define the analog pin for the joystick X-axis
const int joyYPin = A1;   // Define the analog pin for the joystick Y-axis
const int buttonPin = 2;  // Define the digital pin for the joystick button

Servo servoX;
Servo servoY;

bool manualControl = false;  // Flag for manual control
int servoXAngle = 90;        // Current angle of the X-axis servo
int servoYAngle = 90;        // Current angle of the Y-axis servo
int prevServoXAngle = 90;    // Previous angle of the X-axis servo
int prevServoYAngle = 90;    // Previous angle of the Y-axis servo
const int stepSize = 2;      // Step size for smoother servo movement

void setup() {
  Serial.begin(9600);  // Increase baud rate for faster communication
  servoX.attach(servoXPin);  // Attach the X-axis servo to pin 9
  servoY.attach(servoYPin);  // Attach the Y-axis servo to pin 10

  pinMode(buttonPin, INPUT_PULLUP);  // Initialize the button pin as input with pull-up resistor

  servoX.write(90);  // Set initial position for X-axis servo
  servoY.write(90);  // Set initial position for Y-axis servo
}

void loop() {
  // Check if the joystick button is pressed to toggle manual control
  if (digitalRead(buttonPin) == HIGH) {
    manualControl = !manualControl;
    delay(500);  // Debounce delay
  }

  if (manualControl) {
    // Read joystick values
    int joyXValue = analogRead(joyXPin);
    int joyYValue = analogRead(joyYPin);

    // Map joystick values to servo angles (assuming joystick returns values between 0 and 1023)
    int targetXAngle = map(joyXValue, 0, 1023, 180, 0);
    int targetYAngle = map(joyYValue, 0, 1023, 0, 180);

    // Update servo positions smoothly
    updateServoPosition(servoX, servoXAngle, targetXAngle);
    updateServoPosition(servoY, servoYAngle, targetYAngle);

  } else {
    if (Serial.available()) {
      // Read the incoming serial data
      String data = Serial.readStringUntil('\n');
      int commaIndex = data.indexOf(',');

      if (commaIndex > 0) {
        // Extract X and Y coordinates
        int targetXAngle = data.substring(0, commaIndex).toInt();
        int targetYAngle = data.substring(commaIndex + 1).toInt();

        // Update servo positions smoothly
        updateServoPosition(servoX, servoXAngle, targetXAngle);
        updateServoPosition(servoY, servoYAngle, targetYAngle);
      }
    }
  }

  delay(10);  // Small delay for smoother movement
}

void updateServoPosition(Servo &servo, int &currentAngle, int targetAngle) {
  if (currentAngle != targetAngle) {
    if (currentAngle < targetAngle) {
      currentAngle += stepSize;
      if (currentAngle > targetAngle) currentAngle = targetAngle;
    } else {
      currentAngle -= stepSize;
      if (currentAngle < targetAngle) currentAngle = targetAngle;
    }
    servo.write(currentAngle);
  }
}
