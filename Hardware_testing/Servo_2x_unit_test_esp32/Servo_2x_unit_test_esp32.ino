#include <ESP32Servo.h>

Servo servo1;  // Create servo object for first servo
Servo servo2;  // Create servo object for second servo

const int servoPin1 = 14;  // First servo on GPIO 14
const int servoPin2 = 26;  // Second servo on GPIO 26

void setup() {
    servo1.attach(servoPin1);  // Attach first servo
    servo2.attach(servoPin2);  // Attach second servo
    delay(500);
    servo1.write(45);  // Move first servo
    servo2.write(45);
    delay(500);
    servo1.write(90);  // Move first servo
    servo2.write(90);  
}

void loop() {
    servo1.write(90);  // Move first servo
    servo2.write(90);  
}
