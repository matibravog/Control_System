#include <Wire.h>
#include "SparkFun_BNO080_Arduino_Library.h"
#include <Adafruit_BME680.h>
#include <TinyGPSPlus.h>
#include "EBYTE.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define EBYTE_RX 16
#define EBYTE_TX 17
#define EBYTE_M0 4
#define EBYTE_M1 2
#define EBYTE_AX 15

#define GPS_RX 5
#define GPS_TX 18

// Sensor objects
BNO080 imu;
Adafruit_BME680 bme;
TinyGPSPlus gps;

// RF object
EBYTE transceiver(&Serial2, EBYTE_M0, EBYTE_M1, EBYTE_AX);

// Data structs
struct IMUData {
  float roll;
  float pitch;
  float yaw;
  uint32_t timestamp_ms;
};

struct BaroData {
  float pressure_hPa;
  float altitude_m;
  uint32_t timestamp_ms;
};

struct GPSData {
  double lat;
  double lon;
  double altitude_m;
  uint32_t timestamp_ms;
};

struct TelemetryPacket {
  IMUData imu;
  BaroData baro;
  GPSData gps;
};

// Shared data variables
IMUData imuData;
BaroData baroData;
GPSData gpsData;

// Mutex to protect shared data
portMUX_TYPE dataMux = portMUX_INITIALIZER_UNLOCKED;

// Task handles
TaskHandle_t TaskIMU;
TaskHandle_t TaskBaroGPS;  // combine Baro + GPS for simplicity
TaskHandle_t TaskRF;

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  // IMU init
  if (!imu.begin()) {
    Serial.println("ERROR: BNO080 no detectado.");
    while (1) { delay(1000); }
  }
  Serial.println("IMU BNO080 iniciado");
  imu.enableRotationVector(50); // 50 Hz

  // Barometer init
  if (!bme.begin(0x77)) {
    Serial.println("ERROR: BME680 no detectado.");
    while (1) { delay(1000); }
  }
  Serial.println("BARO BME680 iniciado");
  bme.setPressureOversampling(BME680_OS_16X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(0, 0);

  // GPS init
  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS Serial1 iniciado");

  // RF init
  Serial2.begin(9600, SERIAL_8N1, EBYTE_RX, EBYTE_TX);
  transceiver.init();
  Serial.println("EBYTE iniciado");

  // Create FreeRTOS tasks
  xTaskCreatePinnedToCore(TaskReadIMU, "IMU_Task", 4096, NULL, 1, &TaskIMU, 0);
  xTaskCreatePinnedToCore(TaskReadBaroGPS, "BaroGPS_Task", 4096, NULL, 1, &TaskBaroGPS, 1);
  xTaskCreatePinnedToCore(TaskSendRF, "RF_Task", 4096, NULL, 1, &TaskRF, 1);
}

void loop() {
  // Empty - all work done in tasks
}


void TaskReadIMU(void *pvParameters) {
  while (true) {
    if (imu.dataAvailable()) {
      IMUData newData;
      newData.roll = imu.getRoll() * 180.0 / PI;
      newData.pitch = imu.getPitch() * 180.0 / PI;
      newData.yaw = imu.getYaw() * 180.0 / PI;
      newData.timestamp_ms = millis();

      // Protect shared variable update with mutex
      portENTER_CRITICAL(&dataMux);
      imuData = newData;
      portEXIT_CRITICAL(&dataMux);

      Serial.printf("[IMU %lu ms] R=%.1f P=%.1f Y=%.1f\n", newData.timestamp_ms, newData.roll, newData.pitch, newData.yaw);
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // ~20 Hz
  }
}

void TaskReadBaroGPS(void *pvParameters) {
  while (true) {
    // Read GPS chars
    while (Serial1.available()) {
      char c = (char)Serial1.read();
      gps.encode(c);
    }

    // Update GPS data if available
    if (gps.location.isUpdated()) {
      GPSData newGpsData;
      newGpsData.lat = gps.location.lat();
      newGpsData.lon = gps.location.lng();
      newGpsData.altitude_m = gps.altitude.meters();
      newGpsData.timestamp_ms = millis();

      portENTER_CRITICAL(&dataMux);
      gpsData = newGpsData;
      portEXIT_CRITICAL(&dataMux);

      Serial.printf("[GPS %lu ms] lat:%.6f lon:%.6f alt:%.2f\n", newGpsData.timestamp_ms, newGpsData.lat, newGpsData.lon, newGpsData.altitude_m);
    }

    // Barometer reading
    if (bme.performReading()) {
      BaroData newBaroData;
      newBaroData.pressure_hPa = bme.pressure / 100.0;
      newBaroData.altitude_m = 44330.0 * (1.0 - pow(newBaroData.pressure_hPa / 1013.25, 0.1903));
      newBaroData.timestamp_ms = millis();

      portENTER_CRITICAL(&dataMux);
      baroData = newBaroData;
      portEXIT_CRITICAL(&dataMux);

      Serial.printf("[Baro %lu ms] Pressure=%.2f hPa Altitude=%.2f m\n", newBaroData.timestamp_ms, newBaroData.pressure_hPa, newBaroData.altitude_m);
    } else {
      Serial.println("[Baro] Reading failed.");
    }

    vTaskDelay(pdMS_TO_TICKS(100)); // ~10 Hz
  }
}

void TaskSendRF(void *pvParameters) {
  while (true) {
    TelemetryPacket packet;

    // Copy shared data under mutex protection
    portENTER_CRITICAL(&dataMux);
    packet.imu = imuData;
    packet.baro = baroData;
    packet.gps = gpsData;
    portEXIT_CRITICAL(&dataMux);

    // Send via RF
    transceiver.SendStruct(&packet, sizeof(packet));

    Serial.printf("[RF %lu ms] Sent telemetry packet: IMU(R=%.1f, P=%.1f, Y=%.1f), Baro(Alt=%.2f m), GPS(lat=%.6f, lon=%.6f, alt=%.2f m)\n",
                  millis(),
                  packet.imu.roll, packet.imu.pitch, packet.imu.yaw,
                  packet.baro.altitude_m,
                  packet.gps.lat, packet.gps.lon, packet.gps.altitude_m);

    vTaskDelay(pdMS_TO_TICKS(300)); // 20 Hz send rate, adjust if needed
  }
}
