#include <esp_now.h>
#include <WiFi.h>

typedef struct {
    float yaw, pitch;
    float servoYaw, servoPitch;
} SensorData;

SensorData receivedData;

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingData, int len) {
    memcpy(&receivedData, incomingData, sizeof(receivedData));

    // Serial.print("Received data from: ");
    // for (int i = 0; i < 6; i++) {
    //     Serial.print(info->src_addr[i], HEX);
    //     if (i < 5) Serial.print(":");
    // }
    // Serial.println();

    Serial.print(receivedData.yaw);
    Serial.print(","); Serial.print(receivedData.pitch);
    Serial.print(","); Serial.print(receivedData.servoYaw);
    Serial.print(","); Serial.println(receivedData.servoPitch);
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