#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

Adafruit_MPU6050 mpu;

uint8_t receiverMAC[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60}; // Replace with your receiver's MAC

typedef struct {
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
} SensorData;

SensorData sensorData;

void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Send Status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

void setup() {
    Serial.begin(115200);
    Wire.begin();
    
    if (!mpu.begin()) {
        Serial.println("MPU6050 not found!");
        while (1);
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW Init Failed");
        return;
    }
    
    esp_now_register_send_cb(OnSent);
    
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, receiverMAC, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    sensorData.accX = a.acceleration.x;
    sensorData.accY = a.acceleration.y;
    sensorData.accZ = a.acceleration.z;
    sensorData.gyroX = g.gyro.x;
    sensorData.gyroY = g.gyro.y;
    sensorData.gyroZ = g.gyro.z;

    esp_now_send(receiverMAC, (uint8_t *)&sensorData, sizeof(sensorData));

    Serial.print("Sent -> AccX: ");
    Serial.print(sensorData.accX);
    Serial.print(", AccY: ");
    Serial.print(sensorData.accY);
    Serial.print(", AccZ: ");
    Serial.print(sensorData.accZ);
    Serial.print(" | GyroX: ");
    Serial.print(sensorData.gyroX);
    Serial.print(", GyroY: ");
    Serial.print(sensorData.gyroY);
    Serial.print(", GyroZ: ");
    Serial.println(sensorData.gyroZ);

    delay(100);
}
