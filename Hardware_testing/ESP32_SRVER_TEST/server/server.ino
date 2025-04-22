#include <WiFi.h>
#include <WebServer.h>

const char* ssid = "Aeroespacial-Alumnos";
const char* password = "aerofi2021";

WebServer server(80);

void handleRoot() {
    server.send(200, "text/plain", "ESP32 Web Server is working!");
}

void setup() {
    Serial.begin(115200);
    WiFi.begin(ssid, password);

    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    
    Serial.println("\nConnected!");
    Serial.print("ESP32 IP Address: ");
    Serial.println(WiFi.localIP());

    server.on("/", handleRoot);
    server.begin();
}

void loop() {
    server.handleClient();
}
