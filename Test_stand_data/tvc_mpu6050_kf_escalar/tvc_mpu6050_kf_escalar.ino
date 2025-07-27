// mpu 6050 + ESP32 + Servo + FreeRTOS + kalman filter
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <ESP32Servo.h>

// Servo setup
Servo s1;
Servo s2;
const int servoPinPitch = 26; // pitch
const int servoPinYaw = 14;   // yaw

// PID constants
float kp = 40, ki = 0, kd = 0;
float dt = 0.002;

int setpointPitch = 0, setpointYaw = 0;
float previousErrorPitch = 0, previousErrorYaw = 0;
float integralPitch = 0, integralYaw = 0;

// Sensor variables
float RatePitch, RateYaw, RateRoll;
float RateCalibrationYaw = 0, RateCalibrationPitch = 0;
float AccX, AccY, AccZ;
float AngleYaw, AnglePitch;

float kalmanYaw = 0, kalmanUncertaintyYaw = 0.04 * 0.04, kalmanGainYaw = 0;
float kalmanPitch = 0, kalmanUncertaintyPitch = 0.04 * 0.04, kalmanGainPitch = 0;
float kalman1DOutput[] = {0, 0, 0}; // angle, uncertainty, gain

uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Data to send
typedef struct {
  float yaw, pitch;
  float kalmanYaw, kalmanPitch;
  float servoYaw, servoPitch;
} SensorData;

SensorData sensorData;

// FreeRTOS task handles
TaskHandle_t TaskControlHandle;
TaskHandle_t TaskTelemetryHandle;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Kalman Filter
void kalman_1d(float &kalmanState, float &kalmanUncertainty, float kalmanInput, float kalmanMeasurement) {
  kalmanState = kalmanState + 0.004 * kalmanInput;
  kalmanUncertainty = kalmanUncertainty + 0.004 * 0.004 * 0.02 * 0.02;
  float kalmanGain = kalmanUncertainty * 1 / (1 * kalmanUncertainty + 0.04 * 0.04);
  kalmanState = kalmanState + kalmanGain * (kalmanMeasurement - kalmanState);
  kalmanUncertainty = (1 - kalmanGain) * kalmanUncertainty;
  kalman1DOutput[0] = kalmanState;
  kalman1DOutput[1] = kalmanUncertainty;
  kalman1DOutput[2] = kalmanGain;
}

// MPU6050 Read
void gyro_signals() {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();

  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RatePitch = (float)GyroX / 65.5;
  RateYaw = (float)GyroY / 65.5;
  RateRoll = (float)GyroZ / 65.5;

  AccX = (float)AccXLSB / 4096 - 0.03;
  AccY = (float)AccYLSB / 4096 + 0.02;
  AccZ = (float)AccZLSB / 4096 - 0.05;

  AngleYaw = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 180.0 / PI;
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 180.0 / PI;
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

// ======== TASK 1: CONTROL LOOP =========
void TaskControl(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    unsigned long t0 = micros();

    gyro_signals();

    RateYaw -= RateCalibrationYaw;
    RatePitch -= RateCalibrationPitch;

    sensorData.yaw = AngleYaw;
    sensorData.pitch = AnglePitch;

    kalman_1d(kalmanYaw, kalmanUncertaintyYaw, RateYaw, AngleYaw);
    sensorData.kalmanYaw = kalmanYaw = kalman1DOutput[0];

    float outputYaw = PID(setpointYaw, kalmanYaw, integralYaw, previousErrorYaw);
    float servoYaw = map(outputYaw, -1000, 1000, 80, 100);
    sensorData.servoYaw = constrain(servoYaw, 80, 100);
    s2.write(sensorData.servoYaw);

    kalman_1d(kalmanPitch, kalmanUncertaintyPitch, RatePitch, AnglePitch);
    sensorData.kalmanPitch = kalmanPitch = kalman1DOutput[0];


    float outputPitch = PID(setpointPitch, kalmanPitch, integralPitch, previousErrorPitch);
    float servoPitch = map(outputPitch, -1000, 1000, 80, 100);
    sensorData.servoPitch = constrain(servoPitch, 80, 100);
    s1.write(sensorData.servoPitch);

    Serial.print("[Control Task] Duration (us): ");
    Serial.println(micros() - t0);

    vTaskDelay(pdMS_TO_TICKS(10)); // ~100Hz
  }
}

// ======== TASK 2: TELEMETRY =========
void TaskTelemetry(void *pvParameters) {
  (void)pvParameters;

  for (;;) {
    unsigned long t1 = micros();

    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
    if (result == ESP_OK) {
      Serial.println("[Telemetry Task] Data sent successfully");
    } else {
      Serial.println("[Telemetry Task] Send failed");
    }

    Serial.print("[Telemetry Task] Duration (us): ");
    Serial.println(micros() - t1);

    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
  }
}

// ======== SETUP ========
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  delay(250);

  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  // Calibrate gyros
  for (int i = 0; i < 2000; i++) {
    gyro_signals();
    RateCalibrationYaw += RateYaw;
    RateCalibrationPitch += RatePitch;
    delay(1);
  }
  RateCalibrationYaw /= 2000.0;
  RateCalibrationPitch /= 2000.0;

  // Init servos
  s1.attach(servoPinPitch);
  s2.attach(servoPinYaw);
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

  // Create tasks
  xTaskCreatePinnedToCore(TaskControl, "TaskControl", 4096, NULL, 1, &TaskControlHandle, 0);
  xTaskCreatePinnedToCore(TaskTelemetry, "TaskTelemetry", 4096, NULL, 1, &TaskTelemetryHandle, 1);
}

void loop() {
  // Not used in FreeRTOS
}
