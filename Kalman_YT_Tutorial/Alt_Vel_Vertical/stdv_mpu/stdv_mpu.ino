#include <Wire.h>
#include <math.h> // For standard deviation calculation

float RateRoll, RatePitch, RateYaw;
float AngleRoll, AnglePitch;
float AccX, AccY, AccZ;
float AccZInertial;
float VelocityVertical;
float LoopTimer;

#define SAMPLE_SIZE 1000 // Number of samples for standard deviation calculation
float accZSamples[SAMPLE_SIZE];
int sampleIndex = 0;

void gyro_signals(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x1A);
  Wire.write(0x05);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t AccXLSB = Wire.read() << 8 | Wire.read();
  int16_t AccYLSB = Wire.read() << 8 | Wire.read();
  int16_t AccZLSB = Wire.read() << 8 | Wire.read();
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x8);
  Wire.endTransmission();
  Wire.beginTransmission(0x68);
  Wire.write(0x43);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();
  RateRoll = (float)GyroX / 65.5;
  RatePitch = (float)GyroY / 65.5;
  RateYaw = (float)GyroZ / 65.5;
  AccX = (float)AccXLSB / 4096 - 0.03;
  AccY = (float)AccYLSB / 4096 + 0.02;
  AccZ = (float)AccZLSB / 4096 - 0.05;
  AngleRoll = atan(AccY / sqrt(AccX * AccX + AccZ * AccZ)) * 1 / (3.142 / 180);
  AnglePitch = -atan(AccX / sqrt(AccY * AccY + AccZ * AccZ)) * 1 / (3.142 / 180);
}

float calculateStandardDeviation(float data[], int size) {
  float sum = 0.0, mean, standardDeviation = 0.0;

  for (int i = 0; i < size; i++) {
    sum += data[i];
  }
  mean = sum / size;

  for (int i = 0; i < size; i++) {
    standardDeviation += pow(data[i] - mean, 2);
  }
  return sqrt(standardDeviation / size);
}

void setup() {
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
}

void loop() {
  if (sampleIndex < SAMPLE_SIZE) {
    gyro_signals();
    AccZInertial = -sin(AnglePitch * (3.142 / 180)) * AccX +
                   cos(AnglePitch * (3.142 / 180)) * sin(AngleRoll * (3.142 / 180)) * AccY +
                   cos(AnglePitch * (3.142 / 180)) * cos(AngleRoll * (3.142 / 180)) * AccZ;
    AccZInertial = (AccZInertial - 1) * 9.81 * 100;

    // Store AccZInertial readings
    accZSamples[sampleIndex] = AccZInertial;
    sampleIndex++;

    Serial.print("Collecting Sample ");
    Serial.print(sampleIndex);
    Serial.print(": ");
    Serial.println(AccZInertial);

    while (micros() - LoopTimer < 4000)
      ;
    LoopTimer = micros();
  } else {
    // Calculate and print the final standard deviation
    float stdDev = calculateStandardDeviation(accZSamples, SAMPLE_SIZE);
    Serial.print("Final Standard Deviation [cm/s^2]: ");
    Serial.println(stdDev);

    // Stop the loop
    while (1) {
      // Infinite loop to halt execution after the final result
    }
  }
}
