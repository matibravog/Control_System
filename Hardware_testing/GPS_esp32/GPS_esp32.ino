#include <TinyGPSPlus.h>

TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // UART2

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 5, 18);  // RX=4, TX=5
  Serial.println("Iniciando lectura GPS...");
}

void loop() {
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isUpdated()) {
    Serial.print("Latitud: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitud: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitud (m): ");
    Serial.println(gps.altitude.meters());  // <---- Altitude added here

    Serial.print("Fecha: ");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.println(gps.date.year());

    Serial.print("Hora UTC: ");
    Serial.print(gps.time.hour());
    Serial.print(":");
    Serial.print(gps.time.minute());
    Serial.print(":");
    Serial.println(gps.time.second());
    Serial.println();
  }
}
