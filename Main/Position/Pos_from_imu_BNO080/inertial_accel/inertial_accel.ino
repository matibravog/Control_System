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

float ax_inertial, ay_inertial, az_inertial = 0;
float vx_inertial, vy_inertial, vz_inertial = 0;
float px_inertial, py_inertial, pz_inertial = 0;

// Bias para sensores
float ax_bias = 0.0, ay_bias = 0.0, az_bias = 0.0;
float gx_bias = 0.0, gy_bias = 0.0, gz_bias = 0.0;
float mx_bias = 0.0, my_bias = 0.0, mz_bias = 0.0;

// Filtro de paso bajo
float alpha = 0.9;
static float ax_f = 0, ay_f = 0, az_f = 0;

float deg2rad = PI / 180.0;
float rad2deg = 180.0 / PI;

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

// --------- TELEMETRY -------------------- 
uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Data to send
typedef struct {
  float kalmanPitch, kalmanRoll, kalmanYaw;
  float ax_inertial, ay_inertial, az_inertial;
  float vx, vy, vz;
  float px, py, pz;
} SensorData;

SensorData sensorData;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Calibración en reposo
void calibrarIMU(int N = 200) {
  Serial.println("Iniciando calibración, mantén el dispositivo quieto...");

  float ax_sum = 0, ay_sum = 0, az_sum = 0;
  float gx_sum = 0, gy_sum = 0, gz_sum = 0;
  float mx_sum = 0, my_sum = 0, mz_sum = 0;

  for (int i = 0; i < N; i++) {
    while (!myIMU.dataAvailable()) ;  // Esperar nueva data

    ax_sum += myIMU.getRawAccelX() * (9.8 / 4096.0);
    ay_sum += myIMU.getRawAccelY() * (9.8 / 4096.0);
    az_sum += myIMU.getRawAccelZ() * (9.8 / 4096.0);

    gx_sum += myIMU.getRawGyroX() * (1.0 / 16.0) * deg2rad;
    gy_sum += myIMU.getRawGyroY() * (1.0 / 16.0) * deg2rad;
    gz_sum += myIMU.getRawGyroZ() * (1.0 / 16.0) * deg2rad;

    mx_sum += myIMU.getRawMagX() * (1.0 / 16.0);
    my_sum += myIMU.getRawMagY() * (1.0 / 16.0);
    mz_sum += myIMU.getRawMagZ() * (1.0 / 16.0);

    delay(10);
  }

  ax_bias = ax_sum / N;
  ay_bias = ay_sum / N;
  az_bias = az_sum / N;

  gx_bias = gx_sum / N;
  gy_bias = gy_sum / N;
  gz_bias = gz_sum / N;

  mx_bias = mx_sum / N;
  my_bias = my_sum / N;
  mz_bias = mz_sum / N;

  Serial.println("Calibración completa.");
  Serial.print("Accel Bias: "); Serial.print(ax_bias); Serial.print(" "); Serial.print(ay_bias); Serial.print(" "); Serial.println(az_bias);
  Serial.print("Gyro Bias: ");  Serial.print(gx_bias); Serial.print(" "); Serial.print(gy_bias); Serial.print(" "); Serial.println(gz_bias);
  Serial.print("Mag Bias: ");   Serial.print(mx_bias); Serial.print(" "); Serial.print(my_bias); Serial.print(" "); Serial.println(mz_bias);
}


void kalman_2d(BLA::Matrix<3, 1> &u, BLA::Matrix<3, 1> &Z, BLA::Matrix<3, 3> &B) {
  
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

    // Filtro de paso bajo
    ax_f = alpha * ax_f + (1 - alpha) * ax;
    ay_f = alpha * ay_f + (1 - alpha) * ay;
    az_f = alpha * az_f + (1 - alpha) * az;

    //  raw gyroscope data to rad/s 
    float gx = myIMU.getRawGyroX() * (1 / 16.0)*deg2rad;
    float gy = myIMU.getRawGyroY() * (1 / 16.0)*deg2rad;
    float gz = myIMU.getRawGyroZ() * (1 / 16.0)*deg2rad;

    //  raw magnetometer data to μT
    float mx = myIMU.getRawMagX() * (1 / 16.0);
    float my = myIMU.getRawMagY() * (1 / 16.0);
    float mz = myIMU.getRawMagZ() * (1 / 16.0);

    // oriantation [rad]
    float accelPitch = atan2(-ay, az);
    float accelRoll = atan2(ax, sqrt(ay * ay + az * az));
    float magYaw = atan2(-my,mx);
    // float magYaw = atan2(-my*cos(kalmanPitch*PI/180) + mz*sin(kalmanPitch*PI/180), mx*cos(kalmanRoll*PI/180) + my*sin(kalmanPitch*PI/180)*sin(kalmanRoll*PI/180) + mz*cos(kalmanPitch*PI/180)*sin(kalmanRoll*PI/180)) * (180.0 / PI);

    // Update kalman matrices
    BLA::Matrix<3, 1> u = {gx, gy, gz};
    BLA::Matrix<3, 1> Z = {accelPitch, accelRoll, magYaw};

    // Apply Kalman filter
    kalman_2d(u, Z, B);
    
    // control input matrix
    BLA::Matrix<3, 3> B = { 0.05, 0.05*sin(kalmanRoll)*tan(kalmanPitch),    0.05*cos(kalmanRoll)*tan(kalmanPitch),
         0,   0.05*cos(kalmanRoll), -0.05*sin(kalmanRoll),
         0,   0.05*sin(kalmanRoll)/cos(kalmanPitch),    0.05*cos(kalmanRoll)/cos(kalmanPitch) };

    // accel from body frame to inertial frame

    // ax_inertial = ax*cos(kalmanYaw)*cos(kalmanPitch) + ay*(sin(kalmanRoll)*sin(kalmanPitch)*cos(kalmanYaw) - sin(kalmanYaw)*cos(kalmanRoll)) + az*(sin(kalmanRoll)*sin(kalmanYaw) + sin(kalmanPitch)*cos(kalmanRoll)*cos(kalmanYaw));
    // ay_inertial = ax*sin(kalmanYaw)*cos(kalmanPitch) + ay*(sin(kalmanRoll)*sin(kalmanYaw)*sin(kalmanPitch) + cos(kalmanRoll)*cos(kalmanYaw)) + az*(-sin(kalmanRoll)*cos(kalmanYaw)+sin(kalmanYaw)*sin(kalmanPitch)*cos(kalmanRoll));
    // az_inertial = ax*-sin(kalmanPitch) + ay*sin(kalmanRoll)*cos(kalmanPitch) +az*cos(kalmanRoll)*cos(kalmanPitch);

    // Rotar a marco inercial
    ax_inertial = ax_f * cos(kalmanYaw)*cos(kalmanPitch) +
                  ay_f * (sin(kalmanRoll)*sin(kalmanPitch)*cos(kalmanYaw) - sin(kalmanYaw)*cos(kalmanRoll)) +
                  az_f * (sin(kalmanRoll)*sin(kalmanYaw) + sin(kalmanPitch)*cos(kalmanRoll)*cos(kalmanYaw));

    ay_inertial = ax_f * sin(kalmanYaw)*cos(kalmanPitch) +
                  ay_f * (sin(kalmanRoll)*sin(kalmanYaw)*sin(kalmanPitch) + cos(kalmanRoll)*cos(kalmanYaw)) +
                  az_f * (-sin(kalmanRoll)*cos(kalmanYaw)+sin(kalmanYaw)*sin(kalmanPitch)*cos(kalmanRoll));

    az_inertial = ax_f * -sin(kalmanPitch) +
                  ay_f * sin(kalmanRoll)*cos(kalmanPitch) +
                  az_f * cos(kalmanRoll)*cos(kalmanPitch);

    //  Quitar gravedad (solo eje Z en marco inercial)
    az_inertial -= 9.81;

    // Solo integrar si la magnitud es significativa (> umbral)
    float acc_magnitude = sqrt(ax_inertial * ax_inertial +
                              ay_inertial * ay_inertial +
                              az_inertial * az_inertial);

    if (acc_magnitude > 0.02) { // umbral mínimo (ajustable)
      vx_inertial += ax_inertial * 0.05;
      vy_inertial += ay_inertial * 0.05;
      vz_inertial += az_inertial * 0.05;

      px_inertial += vx_inertial * 0.05;
      py_inertial += vy_inertial * 0.05;
      pz_inertial += vz_inertial * 0.05;
    }

    sensorData.ax_inertial = ax_inertial;
    sensorData.ay_inertial = ay_inertial;
    sensorData.az_inertial = az_inertial;

    sensorData.vx = vx_inertial;
    sensorData.vy = vy_inertial;
    sensorData.vz = vz_inertial;
    
    sensorData.px = px_inertial;
    sensorData.py = py_inertial;
    sensorData.pz = pz_inertial;

    // Update telemetry data
    sensorData.kalmanPitch = kalmanPitch*rad2deg;
    sensorData.kalmanRoll = kalmanRoll*rad2deg;
    sensorData.kalmanYaw = magYaw*rad2deg;

    // Print results
    Serial.print(kalmanPitch*rad2deg);
    Serial.print("\t");
    Serial.print(kalmanRoll*rad2deg);
    Serial.print("\t");
    Serial.println(kalmanYaw*rad2deg);

    Serial.print(ax_inertial);
    Serial.print("\t");
    Serial.print(ay_inertial);
    Serial.print("\t");
    Serial.println(az_inertial);
    
    Serial.print(vx_inertial);
    Serial.print("\t");
    Serial.print(vy_inertial);
    Serial.print("\t");
    Serial.println(vz_inertial);

    Serial.print(px_inertial);
    Serial.print("\t");
    Serial.print(py_inertial);
    Serial.print("\t");
    Serial.println(pz_inertial);
  }

  // TELEMETRY
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    // Serial.println("[Telemetry Task] Data sent successfully");
  } else {
    // Serial.println("[Telemetry Task] Send failed");
  }
  delay(50);
}