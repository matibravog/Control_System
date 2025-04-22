#include <WiFi.h>

void setup() {
    Serial.begin(115200);
    delay(100);
    WiFi.mode(WIFI_MODE_STA);
    delay(100);
    Serial.print("ESP32 MAC Address: ");
    Serial.println(WiFi.macAddress());
}

void loop() {
}
