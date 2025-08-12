#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// Must match transmitter definitions exactly
struct __attribute__((packed)) IMUSample {
  int16_t roll;   // degrees *100
  int16_t pitch;
  int16_t yaw;
  uint16_t ts;    // seconds since boot (uint16)
};

struct __attribute__((packed)) BaroSample {
  uint16_t pressure; // hPa *100
  int16_t altitude;  // m *100
  uint16_t ts;       // seconds since boot
};

#define IMU_SAMPLES_PER_PACKET 10

struct __attribute__((packed)) TelemetryPacket {
  uint8_t imu_count;
  IMUSample imu[IMU_SAMPLES_PER_PACKET];
  BaroSample baro;
  uint32_t seq;
};

// Validate angle range -180° to +180°
bool isValidAngle(int16_t val) {
  float angle = val / 100.0f;
  return angle >= -180.0f && angle <= 180.0f;
}

void OnDataRecv(const esp_now_recv_info_t *mac_addr, const uint8_t *data, int len) {
  if (len != sizeof(TelemetryPacket)) {
    Serial.printf("[RX] Packet size mismatch: %d bytes (expected %d)\n", len, sizeof(TelemetryPacket));
    return;
  }

  TelemetryPacket pkt;
  memcpy(&pkt, data, sizeof(pkt));

  Serial.printf("\n[RX] Packet seq=%u\n", pkt.seq);
  Serial.printf(" Baro: Pressure=%.2f hPa, Altitude=%.2f m, TS=%u\n",
                pkt.baro.pressure / 100.0f,
                pkt.baro.altitude / 100.0f,
                pkt.baro.ts);

  for (uint8_t i = 0; i < pkt.imu_count; i++) {
    if (!isValidAngle(pkt.imu[i].roll) || !isValidAngle(pkt.imu[i].pitch) || !isValidAngle(pkt.imu[i].yaw)) {
      Serial.printf(" IMU[%d]: CORRUPTED data! Raw: Roll=%d, Pitch=%d, Yaw=%d, TS=%u\n",
                    i, pkt.imu[i].roll, pkt.imu[i].pitch, pkt.imu[i].yaw, pkt.imu[i].ts);
    } else {
      Serial.printf(" IMU[%d]: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°, TS=%u\n",
                    i,
                    pkt.imu[i].roll / 100.0f,
                    pkt.imu[i].pitch / 100.0f,
                    pkt.imu[i].yaw / 100.0f,
                    pkt.imu[i].ts);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("ESP-NOW Receiver initialized");
}

void loop() {
  delay(500); // Idle, wait for packets
}
