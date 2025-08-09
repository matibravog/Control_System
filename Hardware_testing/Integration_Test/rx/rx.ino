#include <Arduino.h>
#include "EBYTE.h"

// Pines Serial2 para EBYTE
#define EBYTE_RX 16  // conecta al TX del transmisor
#define EBYTE_TX 17  // conecta al RX del transmisor
#define EBYTE_M0 4
#define EBYTE_M1 2
#define EBYTE_AX 15

// Definición de la estructura igual al transmisor
typedef struct __attribute__((packed)) {
  float roll;
  float pitch;
  float yaw;
  float altitude_baro;
  float lat;
  float lon;
  float altitude_gps;
  uint32_t timestamp_ms;
} TelemetryPacket;

TelemetryPacket telemetry;

EBYTE transceiver(&Serial2, EBYTE_M0, EBYTE_M1, EBYTE_AX);

void setup() {
  Serial.begin(115200);
  delay(100);

  Serial2.begin(9600, SERIAL_8N1, EBYTE_RX, EBYTE_TX);

  transceiver.init();
  transceiver.PrintParameters();

  Serial.println("Receptor iniciado");
}

void loop() {
  // Verificar si hay datos disponibles para leer
  if (Serial2.available() >= sizeof(TelemetryPacket)) {
    if (transceiver.GetStruct(&telemetry, sizeof(telemetry))) {
      // Mostrar datos recibidos
      Serial.printf("Recibido:\n");
      Serial.printf("Roll: %.1f, Pitch: %.1f, Yaw: %.1f\n", telemetry.roll, telemetry.pitch, telemetry.yaw);
      Serial.printf("Altitud Baro: %.2f m\n", telemetry.altitude_baro);
      Serial.printf("Latitud: %.6f, Longitud: %.6f\n", telemetry.lat, telemetry.lon);
      Serial.printf("Altitud GPS: %.2f m\n", telemetry.altitude_gps);
      Serial.printf("Timestamp: %u ms\n\n", telemetry.timestamp_ms);
    } else {
      Serial.println("Error leyendo estructura");
    }
  }

  delay(50);
}
