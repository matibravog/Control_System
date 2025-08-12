#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

Adafruit_BME680 bme;

void setup() {
  Serial.begin(115200);

  if (!bme.begin(0x77)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set oversampling to 1x for temperature, pressure, and humidity
  // bme.setTemperatureOversampling(BME680_OS_1X);
  bme.setPressureOversampling(BME680_OS_16X);
  // bme.setHumidityOversampling(BME680_OS_1X);

  // Disable the IIR filter
  bme.setIIRFilterSize(BME680_FILTER_SIZE_0);

  // Disable the gas sensor
  bme.setGasHeater(0, 0); // No gas measurements
}

void loop() {
  unsigned long start = millis();

  // Trigger a forced measurement
  bme.performReading();

  unsigned long end = millis();

  // Print measurement data
  // Serial.print("Temperature = ");
  // Serial.print(bme.temperature);
  // Serial.println(" °C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0); // Convert to hPa
  Serial.println(" hPa");

  // Serial.print("Humidity = ");
  // Serial.print(bme.humidity);
  // Serial.println(" %");

  // Print the time it took for the measurement
  Serial.print("Measurement time: ");
  Serial.print(end - start);
  Serial.println(" ms");

  // Adjust loop timing for your desired sample rate
  delay(1); // Short delay to achieve the highest possible rate
}
