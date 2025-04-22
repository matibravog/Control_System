#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

// Variables for standard deviation
float gyroRollSamples[3000];
float accelRollSamples[3000];
float gyroPitchSamples[3000];
float accelPitchSamples[3000];
int sampleCount = 0;

// Variables for calculation
float gyroRollSum = 0, gyroPitchSum = 0;
float accelRollSum = 0, accelPitchSum = 0;

void setup() {
  Serial.begin(57600);
  Wire.begin();
  Wire.setClock(400000);

  // Initialize MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B); // Power management
  Wire.write(0x00); // Wake up
  Wire.endTransmission();

  delay(500); // Allow MPU to stabilize
}

void loop() {
  if (sampleCount < 1000) { // Collect 1000 samples
    readMPU6050();
    gyroRollSamples[sampleCount] = RateRoll;
    gyroPitchSamples[sampleCount] = RatePitch;
    accelRollSamples[sampleCount] = AngleRoll;
    accelPitchSamples[sampleCount] = AnglePitch;

    // Accumulate sums
    gyroRollSum += RateRoll;
    gyroPitchSum += RatePitch;
    accelRollSum += AngleRoll;
    accelPitchSum += AnglePitch;

    sampleCount++;
  } else {
    // Calculate means
    float gyroRollMean = gyroRollSum / sampleCount;
    float gyroPitchMean = gyroPitchSum / sampleCount;
    float accelRollMean = accelRollSum / sampleCount;
    float accelPitchMean = accelPitchSum / sampleCount;

    // Calculate variances
    float gyroRollVariance = 0, gyroPitchVariance = 0;
    float accelRollVariance = 0, accelPitchVariance = 0;

    for (int i = 0; i < sampleCount; i++) {
      gyroRollVariance += pow(gyroRollSamples[i] - gyroRollMean, 2);
      gyroPitchVariance += pow(gyroPitchSamples[i] - gyroPitchMean, 2);
      accelRollVariance += pow(accelRollSamples[i] - accelRollMean, 2);
      accelPitchVariance += pow(accelPitchSamples[i] - accelPitchMean, 2);
    }

    gyroRollVariance /= sampleCount;
    gyroPitchVariance /= sampleCount;
    accelRollVariance /= sampleCount;
    accelPitchVariance /= sampleCount;

    // Standard deviations
    float gyroRollStdDev = sqrt(gyroRollVariance);
    float gyroPitchStdDev = sqrt(gyroPitchVariance);
    float accelRollStdDev = sqrt(accelRollVariance);
    float accelPitchStdDev = sqrt(accelPitchVariance);

    // Print results
    Serial.println("Standard Deviation Results:");
    Serial.print("Gyro Roll Rate (°/s): ");
    Serial.println(gyroRollStdDev);
    Serial.print("Gyro Pitch Rate (°/s): ");
    Serial.println(gyroPitchStdDev);
    Serial.print("Accel Roll Angle (°): ");
    Serial.println(accelRollStdDev);
    Serial.print("Accel Pitch Angle (°): ");
    Serial.println(accelPitchStdDev);

    // Stop the loop
    while (true);
  }
}

void readMPU6050() {
  // Read accelerometer data
  Wire.beginTransmission(0x68);
  Wire.write(0x3B); // Starting register for Acc
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  int16_t AccXLSB = (Wire.read() << 8 | Wire.read());
  int16_t AccYLSB = (Wire.read() << 8 | Wire.read());
  int16_t AccZLSB = (Wire.read() << 8 | Wire.read());

  // Convert to g
  AccX = (float) AccXLSB/4096 - 0.03;
  AccY = (float) AccYLSB/4096 + 0.02;
  AccZ = (float) AccZLSB/4096 - 0.05;

  // Calculate angles
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * (180.0 / 3.141592653589793);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * (180.0 / 3.141592653589793);

  // Read gyroscope data
  Wire.beginTransmission(0x68);
  Wire.write(0x43); // Starting register for Gyro
  Wire.endTransmission(false);
  Wire.requestFrom(0x68, 6, true);
  int16_t GyroX = (Wire.read() << 8 | Wire.read());
  int16_t GyroY = (Wire.read() << 8 | Wire.read());
  int16_t GyroZ = (Wire.read() << 8 | Wire.read());

  // Convert to °/s
  RateRoll = GyroX / 65.5;
  RatePitch = GyroY / 65.5;
  RateYaw = GyroZ / 65.5;
}
