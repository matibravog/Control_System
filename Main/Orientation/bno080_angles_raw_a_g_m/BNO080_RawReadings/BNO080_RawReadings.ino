
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "SparkFun_BNO080_Arduino_Library.h" // Click here to get the library: http://librarymanager/All#SparkFun_BNO080
BNO080 myIMU;

// --------- TELEMETRY -------------------- 
uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Data to send
typedef struct {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float myaw1, myaw2;
} SensorData;

SensorData sensorData;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup(){
    Serial.begin(115200);
    Serial.println();
    Serial.println("BNO080 Read Example");

    Wire.begin();

    myIMU.begin();

    Wire.setClock(400000); //Increase I2C data rate to 400kHz

    myIMU.enableAccelerometer(50);    //We must enable the accel in order to get MEMS readings even if we don't read the reports.
    myIMU.enableRawAccelerometer(50); //Send data update every 50ms
    myIMU.enableGyro(50);
    myIMU.enableRawGyro(50);
    myIMU.enableMagnetometer(50);
    myIMU.enableRawMagnetometer(50);

    Serial.println(F("Raw MEMS readings enabled"));
    Serial.println(F("Output is: (accel) x y z (gyro) x y z (mag) x y z"));

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
}


void loop()
{
  //Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    // Convert raw accelerometer data to m/s²
    float ax = myIMU.getRawAccelX() * (9.8 / 4096.0);
    float ay = myIMU.getRawAccelY() * (9.8 / 4096.0);
    float az = myIMU.getRawAccelZ() * (9.8 / 4096.0);

    // Convert raw gyroscope data to rad/s
    float gx = myIMU.getRawGyroX() * (1 / 16.0) * (PI / 180.0);
    float gy = myIMU.getRawGyroY() * (1 / 16.0) * (PI / 180.0);
    float gz = myIMU.getRawGyroZ() * (1 / 16.0) * (PI / 180.0);

    // Convert raw magnetometer data to Tesla
    float mx = myIMU.getRawMagX() * (1 / 16.0);
    float my = myIMU.getRawMagY() * (1 / 16.0);
    float mz = myIMU.getRawMagZ() * (1 / 16.0);


    // Calculate pitch, roll, and yaw using accelerometer
    float accelPitch = atan2(-ay, az) * (180.0 / PI);
    float accelRoll = atan2(ax, sqrt(ay * ay + az * az)) * (180.0 / PI);
    float accelYaw = 0; // Accelerometer cannot determine yaw
    
    // Calculate pitch, roll, and yaw using gyroscope (integrate over time)
    static float gyroPitch = 0, gyroRoll = 0, gyroYaw = 0;
    // static unsigned long lastTime = millis();
    // unsigned long currentTime = millis();
    // float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    // lastTime = currentTime;

    gyroPitch = gyroPitch - gx * (0.05) * (180.0 / PI);
    gyroRoll = gyroRoll - gy * (0.05)  * (180.0 / PI);
    gyroYaw = gyroYaw + gz * (0.05)  * (180.0 / PI);

    // Calculate  yaw using magnetometer
    float magYaw_1 = atan2(-my*cos(accelPitch*PI/180) + mz*sin(accelPitch*PI/180), 
                            mx*cos(accelRoll*PI/180) + my*sin(accelPitch*PI/180)*sin(accelRoll*PI/180) + mz*cos(accelPitch*PI/180)*sin(accelRoll*PI/180)) * (180.0 / PI);
    float magYaw_2 = atan2(-my,mx) * (180.0 / PI);
    
    // sensorData.ax = ax;
    // sensorData.ay = ay;
    // sensorData.az = az;
    
    // sensorData.gx = gx;
    // sensorData.gy = gy;
    // sensorData.gz = gz;

    // sensorData.mx = mx;
    // sensorData.my = my;
    // sensorData.mz = mz;

    // sensorData.myaw1 = magYaw_1;
    // sensorData.myaw2 = magYaw_2;

    sensorData.ax = ax;
    sensorData.ay = ay;
    sensorData.az = az;

    sensorData.gx = 0;
    sensorData.gy = 0;
    sensorData.gz = 0;

    sensorData.mx = mx;
    sensorData.my = my;
    sensorData.mz = mz;

    sensorData.myaw1 = magYaw_1;
    sensorData.myaw2 = magYaw_2;
    
    // // Print the converted values
    Serial.print("Accel (m/s²): ");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);
    Serial.print("\t");

    Serial.print("Gyro (rad/s): ");
    Serial.print(gyroPitch);
    Serial.print("\t");
    Serial.print(gyroRoll);
    Serial.print("\t");
    Serial.print(gyroYaw);
    Serial.print("\t");

    Serial.print("Mag (Tesla): ");
    Serial.print(mx);
    Serial.print("\t");
    Serial.print(my);
    Serial.print("\t");
    Serial.print(mz);
    Serial.print("\t");

    // Print pitch, roll, and yaw for each sensor
    // Serial.print("Accel Pitch (deg): ");
    // Serial.print(accelPitch);
    // Serial.print("\tRoll (deg): ");
    // Serial.print(accelRoll);
    // Serial.print("\tYaw (deg): ");
    // Serial.print(accelYaw);
    // Serial.print("\t");

    // Serial.print("Gyro Pitch (deg): ");
    // Serial.print(gyroPitch);
    // Serial.print("\tRoll (deg): ");
    // Serial.print(gyroRoll);
    // Serial.print("\tYaw (deg): ");
    // Serial.print(gyroYaw);
    // Serial.print("\t");

    // Serial.print("Mag Pitch (deg): ");
    // Serial.print(magPitch);
    // Serial.print("\tRoll (deg): ");
    // Serial.print(magRoll);
    // Serial.print("\tYaw (deg): ");
    // Serial.print(magYaw);
    // Serial.print("\t");

    Serial.println();
  }


  // TELEMETRY 
  esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
  if (result == ESP_OK) {
    Serial.println("[Telemetry Task] Data sent successfully");
  } else {
    Serial.println("[Telemetry Task] Send failed");
  }
  
  // timming
  delay(50);
}
