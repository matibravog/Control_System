#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
#include <BasicLinearAlgebra.h>

// imu object
BNO080 myIMU;

// kalman and timming variables
float LoopTimer;
float kalmanPitch, kalmanRoll, kalmanYaw = 0;
int sample_rate = 20; // 20Hz sample rate
float dt = 1/sample_rate; // s

// --------- TELEMETRY -------------------- 
uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Data to send
typedef struct {
  float kalmanPitch, kalmanRoll, kalmanYaw;
} SensorData;

SensorData sensorData;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

using namespace BLA;
BLA::Matrix<3,1> X; // state vector
BLA::Matrix<3,3> A; // transition matrix
BLA::Matrix<3,3> B; // control matrix
BLA::Matrix<3,3> P; // process uncertainty
BLA::Matrix<3,3> Q; // process noise covariance
BLA::Matrix<3,3> H; // observation matrix
BLA::Matrix<3,3> I; // identity matrix
BLA::Matrix<3,3> R; // sensor noise covariance
BLA::Matrix<3,3> L; // aux-1. variable
BLA::Matrix<3,3> inv_L = Inverse(L); // aux. variable inverted
BLA::Matrix<3, 3> K; // kalman gain
BLA::Matrix<3, 1> Y; // aux-2 variable

void kalman_2d(BLA::Matrix<3, 1> &u, BLA::Matrix<3, 1> &Z) {
  
  // state and uncertinty prediction
  X = A*X + B*u;
  P = A*P*~A + Q;
  
  // difference between meassured and predicted state
  Y = (Z - H*X);
  
  // aux
  L = H*P*~H + R;

  // Kalman gain
  K = P*~H*Inverse(L);

  // state process covariance Update
  X = X + K*Y;
  P = (I - K*H)*P;

  // output
  kalmanPitch = X(0, 0);
  kalmanRoll = X(1, 0);
  kalmanYaw = X(2, 0);

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
  A = {1.00,   0,  0,
       0,   1.00,  0,
       0,   0,  1.00};

  //Control matrix
  B = {0.05, 0, 0,
       0, 0.05, 0,
       0, 0, 0.05};

  //Observation matrix
  H = {1, 0, 0,
       0, 1, 0,
       0, 0, 1};
  //
  I = {1, 0, 0,
       0, 1, 0,
       0, 0, 1};          
  
  // process noise covariance
  Q = B * ~B * 5.0f * 5.0f;
  // measurement noise covariance
  R = {0.5 *0.5,    0,          0,
          0,     0.5 * 0.5,     0,
          0,        0,         0.5 * 0.5};

  // initial kalman state
  X = {0,
       0,
       0};
  // initial process uncertainty
  P = {0, 0,0,
       0, 0, 0,
       0, 0, 0};
  
       
       
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
  // Look for reports from the IMU
  if (myIMU.dataAvailable() == true) {

    //  raw  to m/s²
    float ax = myIMU.getRawAccelX() * (9.8 / 4096.0);
    float ay = myIMU.getRawAccelY() * (9.8 / 4096.0);
    float az = myIMU.getRawAccelZ() * (9.8 / 4096.0);

    // Convert raw gyroscope data to rad/s
    float gx = myIMU.getRawGyroX() * (1 / 16.0) * (PI / 180.0);
    float gy = myIMU.getRawGyroY() * (1 / 16.0) * (PI / 180.0);
    float gz = myIMU.getRawGyroZ() * (1 / 16.0) * (PI / 180.0);

    float mx = myIMU.getRawMagX() * (1 / 16.0);
    float my = myIMU.getRawMagY() * (1 / 16.0);
    float mz = myIMU.getRawMagZ() * (1 / 16.0);

    // Calculate pitch and roll using accelerometer
    float accelPitch = atan2(-ay, az) * (180.0 / PI);
    float accelRoll = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);
    float magYaw = atan2(-my,mx) * (180.0 / PI);

    // Update global matrices
    BLA::Matrix<3, 1> u = {gx, gy, gz};
    BLA::Matrix<3, 1> Z = {accelPitch, accelRoll, magYaw};

    // Apply Kalman filter
    kalman_2d(u, Z);

    // Update telemetry data
    sensorData.kalmanPitch = kalmanPitch;
    sensorData.kalmanRoll = kalmanRoll;
    sensorData.kalmanYaw = magYaw;

    // Print results

    Serial.print(kalmanPitch);
    Serial.print("\t");
    Serial.print(kalmanRoll);
    Serial.print("\t");
    Serial.println(kalmanYaw);
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