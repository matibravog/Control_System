/*
  Using the BNO080 IMU
  By: Nathan Seidle
  SparkFun Electronics
  Date: December 21st, 2017
  SparkFun code, firmware, and software is released under the MIT License.
	Please see LICENSE.md for further details.

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/14586

  This example shows how to output accelerometer values

  Hardware Connections:
  Attach the Qwiic Shield to your Arduino/Photon/ESP32 or other
  Plug the sensor onto the shield
  Serial.print it out at 115200 baud to serial monitor.
*/

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
} SensorData;

SensorData sensorData;

// Callback
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Low-pass filter function
float lowPassFilter(float currentValue, float previousValue, float alpha) {
  return alpha * currentValue + (1 - alpha) * previousValue;
}

void setup()
{
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
  // Static variables to store previous filtered values
  static float filteredAx = 0, filteredAy = 0, filteredAz = 0;
  static float filteredGx = 0, filteredGy = 0, filteredGz = 0;
  static float filteredMx = 0, filteredMy = 0, filteredMz = 0;

  // Alpha value for the low-pass filter (adjust as needed, e.g., 0.1 for smoother filtering)
  const float alpha = 0.5;

  // Look for reports from the IMU
  if (myIMU.dataAvailable() == true)
  {
    // Convert raw accelerometer data to m/s²
    float ax = myIMU.getRawAccelX() * (9.8 / 4096.0);
    float ay = myIMU.getRawAccelY() * (9.8 / 4096.0);
    float az = myIMU.getRawAccelZ() * (9.8 / 4096.0);

    // Apply low-pass filter to accelerometer data
    filteredAx = lowPassFilter(ax, filteredAx, alpha);
    filteredAy = lowPassFilter(ay, filteredAy, alpha);
    filteredAz = lowPassFilter(az, filteredAz, alpha);

    // Convert raw gyroscope data to rad/s
    float gx = myIMU.getRawGyroX() * (1 / 16.0) * (3.14159265359 / 180.0);
    float gy = myIMU.getRawGyroY() * (1 / 16.0) * (3.14159265359 / 180.0);
    float gz = myIMU.getRawGyroZ() * (1 / 16.0) * (3.14159265359 / 180.0);

    // Apply low-pass filter to gyroscope data
    filteredGx = lowPassFilter(gx, filteredGx, alpha);
    filteredGy = lowPassFilter(gy, filteredGy, alpha);
    filteredGz = lowPassFilter(gz, filteredGz, alpha);

    // Convert raw magnetometer data to Tesla
    float mx = myIMU.getRawMagX() * (1 / 16.0);
    float my = myIMU.getRawMagY() * (1 / 16.0);
    float mz = myIMU.getRawMagZ() * (1 / 16.0);

    // Apply low-pass filter to magnetometer data
    filteredMx = lowPassFilter(mx, filteredMx, alpha);
    filteredMy = lowPassFilter(my, filteredMy, alpha);
    filteredMz = lowPassFilter(mz, filteredMz, alpha);

    // Calculate pitch, roll, and yaw using accelerometer
    float accelPitch = atan2(filteredAy, sqrt(filteredAx * filteredAx + filteredAz * filteredAz)) * (180.0 / 3.14159265359);
    float accelRoll = atan2(-filteredAx, filteredAz) * (180.0 / 3.14159265359);
    float accelYaw = 0; // Accelerometer cannot determine yaw

    sensorData.ax = accelPitch;
    sensorData.ay = accelRoll;
    sensorData.az = accelYaw;
    
    // Calculate pitch, roll, and yaw using gyroscope (integrate over time)
    static float gyroPitch = 0, gyroRoll = 0, gyroYaw = 0;
    static unsigned long lastTime = millis();
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - lastTime) / 1000.0; // Convert to seconds
    lastTime = currentTime;

    gyroPitch += filteredGx * deltaTime * (180.0 / 3.14159265359);
    gyroRoll += filteredGy * deltaTime * (180.0 / 3.14159265359);
    gyroYaw += filteredGz * deltaTime * (180.0 / 3.14159265359);

    sensorData.gx = gyroPitch;
    sensorData.gy = gyroRoll;
    sensorData.gz = gyroYaw;

    // Calculate pitch, roll, and yaw using magnetometer
    float magPitch = atan2(-filteredMz, sqrt(filteredMx * filteredMx + filteredMy * filteredMy)) * (180.0 / 3.14159265359);
    float magRoll = atan2(filteredMy, filteredMx) * (180.0 / 3.14159265359);
    float magYaw = atan2(filteredMy, filteredMx) * (180.0 / 3.14159265359);

    sensorData.mx = magPitch;
    sensorData.my = magRoll;
    sensorData.mz = magYaw;
    
    // Print pitch, roll, and yaw for each sensor
    Serial.print("Accel Pitch (deg): ");
    Serial.print(accelPitch);
    Serial.print("\tRoll (deg): ");
    Serial.print(accelRoll);
    Serial.print("\tYaw (deg): ");
    Serial.print(accelYaw);
    Serial.print("\t");

    Serial.print("Gyro Pitch (deg): ");
    Serial.print(gyroPitch);
    Serial.print("\tRoll (deg): ");
    Serial.print(gyroRoll);
    Serial.print("\tYaw (deg): ");
    Serial.print(gyroYaw);
    Serial.print("\t");

    Serial.print("Mag Pitch (deg): ");
    Serial.print(magPitch);
    Serial.print("\tRoll (deg): ");
    Serial.print(magRoll);
    Serial.print("\tYaw (deg): ");
    Serial.print(magYaw);
    Serial.print("\t");

    Serial.println();

    // TELEMETRY 
    esp_err_t result = esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));
    if (result == ESP_OK) {
      Serial.println("[Telemetry Task] Data sent successfully");
    } else {
      Serial.println("[Telemetry Task] Send failed");
    }
    
    // Timing
    delay(50);
  }
}
