#include "BluetoothSerial.h"   // Include Bluetooth library
#include <Arduino.h>           // Include Arduino core library

BluetoothSerial serialBT;      // Create Bluetooth serial object

char incoming_signal;          // Variable to store incoming Bluetooth command

int Speed = 100;               // Initial motor speed

// PWM pins for motor speed control
int right_motor_speed = 14;    // PWM pin for right motor (EnA)
int left_motor_speed = 32;     // PWM pin for left motor (EnB)

// Motor direction control pins
int right_motor_forward = 27;   // Right motor forward pin (IN1)
int right_motor_backward = 26;  // Right motor backward pin (IN2)
int left_motor_forward = 25;    // Left motor forward pin (IN3)
int left_motor_backward = 33;   // Left motor backward pin (IN4)

void setup() {
  Serial.begin(115200);             // Start serial communication for debugging
  serialBT.begin("Aslam-YT");       // Start Bluetooth with name "Aslam YT"

  // Set motor speed control pins as output
  pinMode(right_motor_speed, OUTPUT);
  pinMode(left_motor_speed, OUTPUT);

  // Set motor direction control pins as output
  pinMode(right_motor_forward, OUTPUT);
  pinMode(right_motor_backward, OUTPUT);
  pinMode(left_motor_forward, OUTPUT);
  pinMode(left_motor_backward, OUTPUT);

  // Ensure motors are stopped initially
  stop();
}

void loop() {
  // Check if a Bluetooth command is received
  while (serialBT.available()) {
    incoming_signal = serialBT.read();   // Read the command
    //Serial.println(incoming_signal);     // Print for debugging

    // Speed control commands
    if (incoming_signal == '0') Speed = 100;
    if (incoming_signal == '1') Speed = 110;
    if (incoming_signal == '2') Speed = 120;
    if (incoming_signal == '3') Speed = 130;
    if (incoming_signal == '4') Speed = 140;
    if (incoming_signal == '5') Speed = 150;
    if (incoming_signal == '6') Speed = 180;
    if (incoming_signal == '7') Speed = 200;
    if (incoming_signal == '8') Speed = 220;
    if (incoming_signal == '9') Speed = 240;
    if (incoming_signal == 'q') Speed = 255;

    // Movement commands
    else if (incoming_signal == 'F') forward();         // Forward
    else if (incoming_signal == 'B') backward();        // Backward
    else if (incoming_signal == 'L') left();            // Left
    else if (incoming_signal == 'R') right();           // Right
    else if (incoming_signal == 'S') stop();            // Stop
    else if (incoming_signal == 'I') forward_right();   // Forward-right
    else if (incoming_signal == 'G') forward_left();    // Forward-left
    else if (incoming_signal == 'J') backward_right();  // Backward-right
    else if (incoming_signal == 'H') backward_left();   // Backward-left
  }
}

// Move both motors forward
void forward() {
  analogWrite(right_motor_speed, Speed);
  analogWrite(left_motor_speed, Speed);

  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_backward, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(left_motor_backward, LOW);
}

// Move both motors backward
void backward() {
  analogWrite(right_motor_speed, Speed);
  analogWrite(left_motor_speed, Speed);

  digitalWrite(right_motor_forward, LOW);
  digitalWrite(right_motor_backward, HIGH);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(left_motor_backward, HIGH);
}

// Turn left (right motor forward, left motor backward)
void left() {
  analogWrite(right_motor_speed, Speed);
  analogWrite(left_motor_speed, Speed);

  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_backward, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(left_motor_backward, HIGH);
}

// Turn right (right motor backward, left motor forward)
void right() {
  analogWrite(right_motor_speed, Speed);
  analogWrite(left_motor_speed, Speed);

  digitalWrite(right_motor_forward, LOW);
  digitalWrite(right_motor_backward, HIGH);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(left_motor_backward, LOW);
}

// Stop all motor movement
void stop() {
  analogWrite(right_motor_speed, 0);
  analogWrite(left_motor_speed, 0);

  digitalWrite(right_motor_forward, LOW);
  digitalWrite(right_motor_backward, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(left_motor_backward, LOW);
}

// Move forward and slightly right (right motor stop)
void forward_right() {
  analogWrite(right_motor_speed, 0);  // Stop
  analogWrite(left_motor_speed, Speed);       // Left normal

  digitalWrite(right_motor_forward, LOW);
  digitalWrite(right_motor_backward, LOW);
  digitalWrite(left_motor_forward, HIGH);
  digitalWrite(left_motor_backward, LOW);
}

// Move forward and slightly left (left motor stop)
void forward_left() {
  analogWrite(right_motor_speed, Speed);       // Right normal
  analogWrite(left_motor_speed, 0);    // Left stop

  digitalWrite(right_motor_forward, HIGH);
  digitalWrite(right_motor_backward, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(left_motor_backward, LOW);
}

// Move backward and slightly right (right motor stop)
void backward_right() {
  analogWrite(right_motor_speed, 0);   // Stop slower
  analogWrite(left_motor_speed, Speed);        // Left normal

  digitalWrite(right_motor_forward, LOW);
  digitalWrite(right_motor_backward, LOW);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(left_motor_backward, HIGH);
}

// Move backward and slightly left (left motor stop)
void backward_left() {
  analogWrite(right_motor_speed, Speed);       // Right normal
  analogWrite(left_motor_speed, 0);    // Left stop

  digitalWrite(right_motor_forward, LOW);
  digitalWrite(right_motor_backward, HIGH);
  digitalWrite(left_motor_forward, LOW);
  digitalWrite(left_motor_backward, LOW);
}