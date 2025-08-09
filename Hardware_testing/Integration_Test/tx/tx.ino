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

BNO080 imu;
Adafruit_BME680 bme;
TinyGPSPlus gps;
EBYTE transceiver(&Serial2, EBYTE_M0, EBYTE_M1, EBYTE_AX);

typedef struct {
  float roll;
  float pitch;
  float yaw;
} IMUSample;

typedef struct {
  float altitude_baro;
} BaroSample;

IMUSample imuSamples[20];
BaroSample baroSamples[20];

volatile int imuIndex = 0;
volatile int baroIndex = 0;

TaskHandle_t TaskIMU;
TaskHandle_t TaskBaro;

void setup() {
  Serial.begin(115200);
  delay(100);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  if (!imu.begin()) {
    Serial.println("ERROR: BNO080 no detectado.");
    while (1) { delay(1000); }
  }
  Serial.println("IMU BNO080 iniciado");
  imu.enableRotationVector(50);

  if (!bme.begin(0x77)) {
    Serial.println("ERROR: BME680 no detectado.");
    while (1) { delay(1000); }
  }
  Serial.println("BARO BME680 iniciado");

  bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_1X);
  bme.setHumidityOversampling(BME680_OS_1X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);
  bme.setGasHeater(0, 0); 

  Serial1.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS Serial1 iniciado");

  Serial2.begin(9600, SERIAL_8N1, EBYTE_RX, EBYTE_TX);
  transceiver.init();
  Serial.println("EBYTE iniciado");

  xTaskCreatePinnedToCore(TaskReadIMU, "IMU_Task", 4096, NULL, 1, &TaskIMU, 0);
  xTaskCreatePinnedToCore(TaskReadBaro, "Baro_Task", 4096, NULL, 1, &TaskBaro, 1);
}

void loop() {
  // empty, tasks handle everything
}

void TaskReadIMU(void *pvParameters) {
  while (true) {
    if (imu.dataAvailable()) {
      imuSamples[imuIndex].roll = imu.getRoll() * 180.0 / PI;
      imuSamples[imuIndex].pitch = imu.getPitch() * 180.0 / PI;
      imuSamples[imuIndex].yaw = imu.getYaw() * 180.0 / PI;

      Serial.printf("[IMU] Sample %d: R=%.1f, P=%.1f, Y=%.1f\n",
                    imuIndex, imuSamples[imuIndex].roll, imuSamples[imuIndex].pitch, imuSamples[imuIndex].yaw);

      imuIndex++;
      if (imuIndex >= 20) {
        imuIndex = 0;
        Serial.println("[IMU] 1 second of data collected.");
        // You can do something here with 1s batch, e.g. notify another task or send via RF
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}

void TaskReadBaro(void *pvParameters) {
  while (true) {
    if (bme.performReading()) {
      baroSamples[baroIndex].altitude_baro = bme.readAltitude(1013.25);

      Serial.printf("[Baro] Sample %d: Altitude=%.2f m\n", baroIndex, baroSamples[baroIndex].altitude_baro);

      baroIndex++;
      if (baroIndex >= 20) {
        baroIndex = 0;
        Serial.println("[Baro] 1 second of data collected.");
        // You can do something here with 1s batch, e.g. notify another task or send via RF
      }
    } else {
      Serial.println("[Baro] Reading failed.");
    }
    vTaskDelay(pdMS_TO_TICKS(50)); // 20 Hz
  }
}
