#include <Wire.h>

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch, RateCalibrationYaw;
int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;

uint32_t LoopTimer;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=0.04*0.04;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=0.04*0.04;
float Kalman1DOutput[]={0,0};

float startedTime, elapsedTime, lastTime;

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
  // angle based on gyro rate stdv
  KalmanState = KalmanState + 0.004*KalmanInput;
  //uncertainty based on gyro
  KalmanUncertainty = KalmanUncertainty + 0.004*0.004*0.02*0.02;
  // gain based on previous uncertainty and accel stdv 
  float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 0.04*0.04);
  // new angle
  KalmanState=KalmanState + KalmanGain * (KalmanMeasurement-KalmanState);
  // new uncertainty
  KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;

  Kalman1DOutput[0]=KalmanState; 
  Kalman1DOutput[1]=KalmanUncertainty;
}
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
  int16_t GyroX=Wire.read()<<8 | Wire.read();
  int16_t GyroY=Wire.read()<<8 | Wire.read();
  int16_t GyroZ=Wire.read()<<8 | Wire.read();
  RateRoll=(float)GyroX/65.5;
  RatePitch=(float)GyroY/65.5;
  RateYaw=(float)GyroZ/65.5;
  AccX = (float) AccXLSB/4096 - 0.03;
  AccY = (float) AccYLSB/4096 + 0.02;
  AccZ = (float) AccZLSB/4096 - 0.05;
  AngleRoll=atan(AccY/sqrt(AccX*AccX+AccZ*AccZ))*1/(3.142/180);
  AnglePitch=-atan(AccX/sqrt(AccY*AccY+AccZ*AccZ))*1/(3.142/180);
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

  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    RateCalibrationYaw+=RateYaw;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  
  LoopTimer=micros();
}
void loop() {
  // angles
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;
  // RateYaw-=RateCalibrationYaw;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
  
  Serial.print(KalmanAngleRoll);
  Serial.print(",");
  Serial.println(KalmanAnglePitch);

  while (micros() - LoopTimer < 4000);
  LoopTimer=micros();
}