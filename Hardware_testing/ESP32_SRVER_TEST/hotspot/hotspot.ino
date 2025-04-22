#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "ESP32_Network";
const char* password = "12345678";

WebServer server(80);

void handleRoot() {
    server.send(200, "text/plain", "ESP32 AP Mode is working!");
}

void setup() {
    Serial.begin(115200);
    WiFi.softAP(ssid, password);
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.softAPIP());

    server.on("/", handleRoot);
    server.begin();
}

void loop() {
    server.handleClient();
}
