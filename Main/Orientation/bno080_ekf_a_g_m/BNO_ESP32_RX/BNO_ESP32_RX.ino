#include <esp_now.h>
#include <WiFi.h>

typedef struct {
    float kalmanPitch, kalmanRoll, kalmanYaw;
} SensorData;

SensorData receivedData;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    Serial.print(receivedData.kalmanPitch);Serial.print(","); 
    Serial.print(receivedData.kalmanRoll);Serial.print(","); 
    Serial.println(receivedData.kalmanYaw);
}

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);

    if (esp_now_init() != ESP_OK) {
        Serial.println("ESP-NOW initialization failed!");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}

void loop() {
    // No need to do anything here, everything is handled in the callback
}