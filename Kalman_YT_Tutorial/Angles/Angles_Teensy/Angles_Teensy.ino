#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float AccX, AccY, AccZ;
float AngleRoll, AnglePitch, AngleYaw;

float LoopTimer;
float startedTime, elapsedTime, lastTime;

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
  
  Wire.requestFrom(0x68,6);
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
  
  Wire.requestFrom(0x68,6);
  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  RateRoll = (float)GyroX/65.5;
  RatePitch = (float)GyroY/65.5;
  RateYaw = (float)GyroZ/65.5;

  AccX = (float) AccXLSB/4096 - 0.03;
  AccY = (float) AccYLSB/4096 + 0.02;
  AccZ = (float) AccZLSB/4096 - 0.05;

  AngleRoll = atan(AccY/sqrt(AccX*AccX + AccZ*AccZ))*1/(3.142/180);
  AnglePitch = atan(AccX/sqrt(AccY*AccY + AccZ*AccZ))*1/(3.142/180);
  // AngleYaw = atan(sqrt(AccX*AccX + AccY*AccY)/AccZ)*1/(3.142/180);
}

void setup() {
  // put your main code here, to run repeatedly:
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13,HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  startedTime = micros();
}

void loop() {
  // put your main code here, to run repeatedly:
  gyro_signals();
  // Serial.print("Roll angle []: ");
  // Serial.print(AngleRoll);
  // Serial.print("  Pitch angle []: ");
  // Serial.println(AnglePitch);

  Serial.print(AngleRoll);
  Serial.print(",");
  Serial.println(AnglePitch);

  // manual accel calibration, meassured in G's
  // Serial.print("Ax: ");
  // Serial.print(AccX);
  // Serial.print("   Ay: ");
  // Serial.print(AccY);
  // Serial.print("   Az: ");
  // Serial.println(AccZ);

  // meassuring iteration length
  // Serial.print("time: ");
  // float timer = micros()-startedTime;
  // elapsedTime = timer - lastTime;
  // lastTime = timer;
  // Serial.println(elapsedTime);
}
