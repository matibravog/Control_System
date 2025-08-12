#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_BME680.h>

// Pins
#define I2C_SDA 21
#define I2C_SCL 22

// Receiver MAC (change to your receiver)
uint8_t receiverMac[] = {0x5C, 0x01, 0x3B, 0x65, 0xDD, 0x60};

// Constants
#define IMU_SAMPLES_PER_PACKET 10
#define IMU_SAMPLE_INTERVAL_MS 50  // 20 Hz (10 samples in 500 ms)

// Structs for samples
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

struct __attribute__((packed)) TelemetryPacket {
  uint8_t imu_count;
  IMUSample imu[IMU_SAMPLES_PER_PACKET];
  BaroSample baro;
  uint32_t seq;
};

BNO080 imu;
Adafruit_BME680 bme;

// Mutexes for shared data
SemaphoreHandle_t imuMutex;
SemaphoreHandle_t baroMutex;

// Shared data buffers
IMUSample imuSamples[IMU_SAMPLES_PER_PACKET];
uint8_t imuIndex = 0;

BaroSample latestBaro;

uint32_t seq = 0;

inline uint16_t sec16() { return (uint16_t)((millis() / 1000) & 0xFFFF); }

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("[ESP-NOW] send status: %s\n", status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

// IMU Task: samples IMU every 50 ms, stores 10 samples circularly
void IMU_Task(void *pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(IMU_SAMPLE_INTERVAL_MS);

  for (;;) {
    if (imu.dataAvailable()) {
      float roll_f = imu.getRoll() * 180.0 / PI;
      float pitch_f = imu.getPitch() * 180.0 / PI;
      float yaw_f = imu.getYaw() * 180.0 / PI;

      IMUSample s;
      s.roll = (int16_t)(roll_f * 100.0f);
      s.pitch = (int16_t)(pitch_f * 100.0f);
      s.yaw = (int16_t)(yaw_f * 100.0f);
      s.ts = sec16();

      Serial.printf("[IMU] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°, TS: %u\n",
                    roll_f, pitch_f, yaw_f, s.ts);

      xSemaphoreTake(imuMutex, portMAX_DELAY);
      imuSamples[imuIndex] = s;
      imuIndex = (imuIndex + 1) % IMU_SAMPLES_PER_PACKET;
      xSemaphoreGive(imuMutex);
    }
    vTaskDelay(delayTicks);
  }
}

// Baro Task: samples Barometer every 500 ms
void Baro_Task(void *pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(500);

  for (;;) {
    if (bme.performReading()) {
      float pressure_hPa = bme.pressure / 100.0f;
      float alt = 44330.0 * (1.0 - pow(bme.pressure / 101325.0, 0.1903));

      Serial.printf("[BARO] Pressure=%.2f hPa, Altitude=%.2f m\n", pressure_hPa, alt);

      BaroSample b;
      b.pressure = (uint16_t)(pressure_hPa * 100.0f);
      b.altitude = (int16_t)(alt * 100.0f);
      b.ts = sec16();

      xSemaphoreTake(baroMutex, portMAX_DELAY);
      latestBaro = b;
      xSemaphoreGive(baroMutex);
    } else {
      Serial.println("[BARO] performReading FAILED");
    }
    vTaskDelay(delayTicks);
  }
}

// PacketBuilder Task: builds and sends packet every 500 ms
void PacketBuilder_Task(void *pvParameters) {
  const TickType_t delayTicks = pdMS_TO_TICKS(500);

  for (;;) {
    TelemetryPacket pkt;
    memset(&pkt, 0, sizeof(pkt));
    pkt.seq = seq++;

    xSemaphoreTake(imuMutex, portMAX_DELAY);
    // imuIndex points to next write pos; oldest sample is imuIndex
    for (uint8_t i = 0; i < IMU_SAMPLES_PER_PACKET; i++) {
      uint8_t idx = (imuIndex + i) % IMU_SAMPLES_PER_PACKET;
      pkt.imu[i] = imuSamples[idx];
    }
    pkt.imu_count = IMU_SAMPLES_PER_PACKET;
    xSemaphoreGive(imuMutex);

    xSemaphoreTake(baroMutex, portMAX_DELAY);
    pkt.baro = latestBaro;
    xSemaphoreGive(baroMutex);

    esp_err_t res = esp_now_send(receiverMac, (uint8_t *)&pkt, sizeof(pkt));
    if (res != ESP_OK) {
      Serial.printf("[PacketBuilder] esp_now_send error: %d\n", res);
    } else {
      Serial.printf("[PacketBuilder] Sent packet seq=%u, Baro P=%.2f hPa Alt=%.2f m\n",
                    pkt.seq,
                    pkt.baro.pressure / 100.0f,
                    pkt.baro.altitude / 100.0f);

      for (uint8_t i = 0; i < pkt.imu_count; i++) {
        Serial.printf("  IMU[%u]: Roll=%.2f°, Pitch=%.2f°, Yaw=%.2f°, TS=%u\n",
                      i,
                      pkt.imu[i].roll / 100.0f,
                      pkt.imu[i].pitch / 100.0f,
                      pkt.imu[i].yaw / 100.0f,
                      pkt.imu[i].ts);
      }
    }

    vTaskDelay(delayTicks);
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);

  imuMutex = xSemaphoreCreateMutex();
  baroMutex = xSemaphoreCreateMutex();
  if (!imuMutex || !baroMutex) {
    Serial.println("Failed to create mutexes");
    while (1) delay(1000);
  }

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!imu.begin()) {
    Serial.println("BNO080 init failed");
    while (1) delay(1000);
  }
  imu.enableRotationVector(50);

  if (!bme.begin(0x77)) {
    Serial.println("BME680 init failed");
    while (1) delay(1000);
  }
  bme.setPressureOversampling(BME680_OS_16X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(0, 0);

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }
  esp_now_register_send_cb(OnDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  xTaskCreatePinnedToCore(IMU_Task, "IMU", 4096, NULL, 3, NULL, 1);       // Core 1
  xTaskCreatePinnedToCore(Baro_Task, "BARO", 3072, NULL, 2, NULL, 1);     // Core 1
  xTaskCreatePinnedToCore(PacketBuilder_Task, "PKT_BUILDER", 4096, NULL, 2, NULL, 0);  // Core 0

  Serial.println("Transmitter initialized, running on 2 cores");
}

void loop() {
  vTaskDelay(portMAX_DELAY);
}
