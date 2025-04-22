// mpu 6050 + ESP32 + Servo + FreeRTOS + kalman filter
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>
#include "SparkFun_BNO080_Arduino_Library.h"

BNO080 imu;

// Servo setup
Servo s1; // roll
Servo s2; // pitch

const int servoPinPitch = 26; // pitch
const int servoPinRoll = 14;   // Roll

// PID constants
float kp = 40, ki = 0, kd = 0;
float dt = 0.002;

int setpointPitch = 0, setpointRoll = 0;
float previousErrorPitch = 0, previousErrorRoll = 0;
float integralPitch = 0, integralRoll = 0;

// Sensor variables
float roll, pitch;

uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Data to send
typedef struct {
  float roll, pitch;
  float servoRoll, servoPitch;
} SensorData;

SensorData sensorData;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// PID
float PID(float setpoint, float angle, float &integral, float &prevError) {
  float error = setpoint - angle;
  integral += error * dt;
  float derivative = (error - prevError) / dt;
  float output = kp * error + ki * integral + kd * derivative;
  prevError = error;
  return output;
}

unsigned long lastControlTime = 0;
unsigned long lastTelemetryTime = 0;

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  // Init IMU
  if (imu.begin() == false) {
    Serial.println(F("BNO080 not detected at default I2C address. Check your jumpers and the hookup guide. Freezing..."));
    while (1);
  }
  imu.enableRotationVector(50); // Send data update every 50ms
  Serial.println(F("Rotation vector enabled"));
  Serial.println(F("Output in form roll, pitch, Roll"));
  delay(500); // Wait for BNO to boot

  // Init servos
  s1.attach(servoPinPitch);
  s2.attach(servoPinRoll);
  s1.write(90);
  s2.write(90);

  // Init WiFi + ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  esp_now_register_send_cb(OnSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  unsigned long currentTime = millis();

  // Control task (~20Hz)
  if (imu.dataAvailable() == true) {
    roll = imu.getRoll() * 180.0 / PI; // Convert roll to degrees
    pitch = imu.getPitch() * 180.0 / PI; // Convert pitch to degrees
  }

  sensorData.roll = roll;
  sensorData.pitch = pitch;

  float outputRoll = PID(setpointRoll, roll, integralRoll, previousErrorRoll);
  float servoRoll = map(outputRoll, -1000, 1000, 80, 100);
  sensorData.servoRoll = constrain(servoRoll, 80, 100);
  s2.write(sensorData.servoRoll);

  float outputPitch = PID(setpointPitch, pitch, integralPitch, previousErrorPitch);
  float servoPitch = map(outputPitch, -1000, 1000, 80, 100);
  sensorData.servoPitch = constrain(servoPitch, 80, 100);
  s1.write(sensorData.servoPitch);

  Serial.print("[Control Task] Roll: ");
  Serial.print(roll);
  Serial.print(", Pitch: ");
  Serial.println(pitch);

  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("[Telemetry Task] Data sent successfully");
  } else {
    Serial.println("[Telemetry Task] Send failed");
  }
  delay(10);
}