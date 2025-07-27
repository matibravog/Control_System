#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <BasicLinearAlgebra.h>
BNO080 myIMU;

// --------- TELEMETRY -------------------- 
uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Data to send
typedef struct {
  float kalmanPitch, kalmanRoll;
} SensorData;

SensorData sensorData;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

float LoopTimer;

float kalmanPitch=0, KalmanUncertaintyAnglePitch=0.04*0.04;
float kalmanRoll=0, KalmanUncertaintyAngleRoll=0.04*0.04;
float Kalman1DOutput[]={0,0};

using namespace BLA;
BLA::Matrix<2,1> S; 
BLA::Matrix<2,2> F; 
BLA::Matrix<2,2> G;
BLA::Matrix<2,2> P; 
BLA::Matrix<2,2> Q;
BLA::Matrix<1,0> A;
BLA::Matrix<2,2> H; // Corrected dimensions of H
BLA::Matrix<2,2> I; 
BLA::Matrix<2,2> R; // Corrected dimensions of R
BLA::Matrix<2,2> L; // Corrected dimensions of L
BLA::Matrix<2,2> inv_L = Inverse(L);

void kalman_2d(BLA::Matrix<2, 1> &rates, BLA::Matrix<2, 1> &M) {
  S = F*S + G*rates;
  P = F*P*~F + Q;

  // Corrected L calculation to match dimensions
  L = H*P*~H + R;

  // Calculate Kalman gain (K is a 2x2 matrix)
  BLA::Matrix<2, 2> K = P*~H*Inverse(L);

  // Update state estimate
  S = S + K*(M - H*S);

  // Update global variables for pitch and roll
  kalmanPitch = S(0, 0);
  kalmanRoll = S(1, 0);

  // Update process covariance
  P = (I - K*H)*P;
}

void setup() {
  //initialize teensy
  Serial.begin(115200);
  Wire.begin();
  myIMU.begin();
  Wire.setClock(400000);
  delay(250);

  myIMU.enableRotationVector(50); //Send data update every 50ms
  myIMU.enableAccelerometer(50);    
  myIMU.enableRawAccelerometer(50);
  myIMU.enableGyro(50);
  myIMU.enableRawGyro(50);
  myIMU.enableMagnetometer(50);
  myIMU.enableRawMagnetometer(50);

  
  // --------------------------------------
  
  // kalman matrix
  //state transition matrix
  F = {1,   0,
       0,   1   };  
  // control input matrix
  G = { 0.05,    0,
         0,     0.05 };
  //Observation matrix
  H = {1, 0,
       0, 1};
  //
  I = {1, 0,
       0, 1};
  
  // process noise covariance
  Q = G * ~G * 0.5f * 0.5f;
  // measurement noise covariance
  R = {0.5 * 0.5, 0,
       0, 0.5 * 0.5};

  // initial kalman state
  S = {0,
       0};
  // initial process uncertainty
  P = {0, 0,
       0, 0};
  
       
       
       //-------------------------------------------------
       
  // INIT ESP-NOW PROTOCOL
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }

  // ESP-NOW / Register the send callback function
  esp_now_register_send_cb(OnSent);
  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  LoopTimer=micros();
}

void loop() {
  static unsigned long lastTime = micros(); // Store the last loop time
  unsigned long currentTime = micros();    // Get the current time
  float deltaTime = (currentTime - lastTime) / 1000000.0; // Convert to seconds
  lastTime = currentTime;                  // Update the last loop time

  // Look for reports from the IMU
  if (myIMU.dataAvailable() == true) {
    
    // Convert raw accelerometer data to m/s²
    float ax = myIMU.getRawAccelX() * (9.8 / 4096.0);
    float ay = myIMU.getRawAccelY() * (9.8 / 4096.0);
    float az = myIMU.getRawAccelZ() * (9.8 / 4096.0);

    // Convert raw gyroscope data to rad/s
    float gx = myIMU.getRawGyroX() * (1 / 16.0) * (PI / 180.0);
    float gy = myIMU.getRawGyroY() * (1 / 16.0) * (PI / 180.0);

    // Calculate pitch and roll using accelerometer
    float accelPitch = atan2(ay, sqrt(ax * ax + az * az)) * (180.0 / PI);
    float accelRoll = atan2(-ax, az) * (180.0 / PI);

    // Update global matrices
    BLA::Matrix<2, 1> rates = {gx, gy};
    BLA::Matrix<2, 1> M = {accelPitch, accelRoll};

    // Apply Kalman filter
    kalman_2d(rates, M);

    // Update telemetry data
    sensorData.kalmanPitch = kalmanPitch;
    sensorData.kalmanRoll = kalmanRoll;

    // Print results
    Serial.print(kalmanPitch);
    Serial.print("\t");
    Serial.println(kalmanRoll);
  }

  // TELEMETRY
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("[Telemetry Task] Data sent successfully");
  } else {
    Serial.println("[Telemetry Task] Send failed");
  }
  delay(50);
}