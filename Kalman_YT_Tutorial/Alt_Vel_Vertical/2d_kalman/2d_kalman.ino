#include <Wire.h>
#include <BasicLinearAlgebra.h>

float RateRoll, RatePitch, RateYaw;
float RateCalibrationRoll, RateCalibrationPitch;
int RateCalibrationNumber;

float AccX, AccY, AccZ;
float AngleRoll, AnglePitch;
float AccZInertial;

float LoopTimer;

uint16_t dig_T1, dig_P1;
int16_t  dig_T2, dig_T3, dig_P2, dig_P3, dig_P4, dig_P5;
int16_t  dig_P6, dig_P7, dig_P8, dig_P9; 
float AltitudeBarometer, AltitudeBarometerStartUp;

float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=0.04*0.04;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=0.04*0.04;
float Kalman1DOutput[]={0,0};

using namespace BLA;
float AltitudeKalman, VelocityVerticalKalman;
BLA::Matrix<2,2> F; BLA::Matrix<2,1> G;
BLA::Matrix<2,2> P; BLA::Matrix<2,2> Q;
BLA::Matrix<2,1> S; BLA::Matrix<1,2> H;
BLA::Matrix<2,2> I; BLA::Matrix<1,1> Acc;
BLA::Matrix<2,1> K; BLA::Matrix<1,1> R;
BLA::Matrix<1,1> L; BLA::Matrix<1,1> M;
BLA::Matrix<1,1> inv_L = Inverse(L);

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
void kalman_2d(void){

  // if (AccZInertial < 0.1){
  //   AccZInertial = 0;
  // }

  Acc = {AccZInertial};
  M = {AltitudeBarometer};

  S = F*S + G*Acc;
  P = F*P*~F + Q;
  L = H*P*~H + R;
  K = P*~H*inv_L;  
  
  S = S + K*(M - H*S);
  AltitudeKalman = S(0,0); 
  VelocityVerticalKalman = S(1,0); 
  
  P = (I-K*H)*P;
}
void barometer_signals(void){
  Wire.beginTransmission(0x76);
  Wire.write(0xF7);
  Wire.endTransmission();
  Wire.requestFrom(0x76,6);
  uint32_t press_msb = Wire.read();
  uint32_t press_lsb = Wire.read();
  uint32_t press_xlsb = Wire.read();
  uint32_t temp_msb = Wire.read();
  uint32_t temp_lsb = Wire.read();
  uint32_t temp_xlsb = Wire.read();
  unsigned long int adc_P = (press_msb << 12) | (press_lsb << 4) | (press_xlsb >>4);
  unsigned long int adc_T = (temp_msb << 12) | (temp_lsb << 4) | (temp_xlsb >>4);
  signed long int var1, var2;
  var1 = ((((adc_T >> 3) - ((signed long int )dig_T1 <<1)))* ((signed long int )dig_T2)) >> 11;
  var2 = (((((adc_T >> 4) - ((signed long int )dig_T1)) * ((adc_T>>4) - ((signed long int )dig_T1)))>> 12) * ((signed long int )dig_T3)) >> 14;
  signed long int t_fine = var1 + var2;
  unsigned long int p;
  var1 = (((signed long int )t_fine)>>1) - (signed long int )64000;
  var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int )dig_P6);
  var2 = var2 + ((var1*((signed long int )dig_P5)) <<1);
  var2 = (var2>>2)+(((signed long int )dig_P4) <<16);
  var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13))>>3)+((((signed long int )dig_P2) * var1)>>1))>>18;
  var1 = ((((32768+var1))*((signed long int )dig_P1)) >>15);
  if (var1 == 0) { p=0;}    
  p = (((unsigned long int )(((signed long int ) 1048576)-adc_P)-(var2>>12)))*3125;
  if(p<0x80000000){ p = (p << 1) / ((unsigned long int ) var1);}
  else { p = (p / (unsigned long int )var1) * 2;  }
  var1 = (((signed long int )dig_P9) * ((signed long int ) (((p>>3) * (p>>3))>>13)))>>12;
  var2 = (((signed long int )(p>>2)) * ((signed long int )dig_P8))>>13;
  p = (unsigned long int)((signed long int )p + ((var1 + var2+ dig_P7) >> 4));
  double pressure=(double)p/100;
  AltitudeBarometer=44330*(1-pow(pressure/1013.25, 1/5.255))*100;
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
  int16_t AccZLSB = Wire.read() << 8 |  Wire.read();
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
  //initialize teensy
  Serial.begin(57600);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Wire.setClock(400000);
  Wire.begin();
  delay(250);

  //initialize mpu6050
  Wire.beginTransmission(0x68); 
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  //initialize bmp280
  Wire.beginTransmission(0x76);
  Wire.write(0xF4);
  Wire.write(0x57);
  Wire.endTransmission();   

  Wire.beginTransmission(0x76);
  Wire.write(0xF5); 
  Wire.write(0x14);
  Wire.endTransmission();   

  uint8_t data[24], i=0;
  Wire.beginTransmission(0x76);
  Wire.write(0x88);
  Wire.endTransmission();
  Wire.requestFrom(0x76,24);      

  while(Wire.available()){
    data[i] = Wire.read();
    i++;
  } 
  dig_T1 = (data[1] << 8) | data[0]; 
  dig_T2 = (data[3] << 8) | data[2];
  dig_T3 = (data[5] << 8) | data[4];
  dig_P1 = (data[7] << 8) | data[6]; 
  dig_P2 = (data[9] << 8) | data[8];
  dig_P3 = (data[11]<< 8) | data[10];
  dig_P4 = (data[13]<< 8) | data[12];
  dig_P5 = (data[15]<< 8) | data[14];
  dig_P6 = (data[17]<< 8) | data[16];
  dig_P7 = (data[19]<< 8) | data[18];
  dig_P8 = (data[21]<< 8) | data[20];
  dig_P9 = (data[23]<< 8) | data[22]; delay(250);

  //Calibrate bmp280
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    barometer_signals();
    AltitudeBarometerStartUp+=AltitudeBarometer; 
    delay(1); 
  }

  AltitudeBarometerStartUp/=2000;

  //calibrate gyroscope
  for (RateCalibrationNumber=0; RateCalibrationNumber<2000; RateCalibrationNumber ++) {
    gyro_signals();
    RateCalibrationRoll+=RateRoll;
    RateCalibrationPitch+=RatePitch;
    delay(1);
  }
  RateCalibrationRoll/=2000;
  RateCalibrationPitch/=2000;
  
  // kalman matrix

  //state transition matrix
  F = {1, 0.004,
       0,   1   };  
  // control input matrix
  G = {0.5*0.004*0.004,
          0.004        };
  //Observation matrix
  H = {1, 0};
  //
  I = {1, 0,
       0, 1};
  
  // process noise covariance
  Q = G * ~G * 200.0f * 200.0f;
  // measurement noise covariance
  R = {200*200};

  // initial process uncertainty
  P = {0, 0,
       0, 0};
  // initial kalman state
  S = {0,
       0};
  
  LoopTimer=micros();
}
void loop() {
  gyro_signals();
  RateRoll-=RateCalibrationRoll;
  RatePitch-=RateCalibrationPitch;

  kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, RateRoll, AngleRoll);
  KalmanAngleRoll=Kalman1DOutput[0]; 
  KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
  
  kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, RatePitch, AnglePitch);
  KalmanAnglePitch=Kalman1DOutput[0]; 
  KalmanUncertaintyAnglePitch=Kalman1DOutput[1];

  // AccZInertial = -sin(AnglePitch*(3.142/180))*AccX + cos(AnglePitch*(3.142/180))*sin(AngleRoll*(3.142/180))*AccY + cos(AnglePitch*(3.142/180))*cos(AngleRoll*(3.142/180))*AccZ;   
  AccZInertial = -sin(KalmanAnglePitch*(3.142/180))*AccX + cos(KalmanAnglePitch*(3.142/180))*sin(KalmanAngleRoll*(3.142/180))*AccY + cos(KalmanAnglePitch*(3.142/180))*cos(KalmanAngleRoll*(3.142/180))*AccZ;   
  AccZInertial=(AccZInertial-1)*9.81*100;

  barometer_signals();
  AltitudeBarometer-=AltitudeBarometerStartUp;
  
  kalman_2d();
  // Serial.print("Altitude [cm]: ");
  // Serial.print(" Vertical velocity [cm/s]: ");
  // Serial.print(K);
  // Serial.print(",");
  // Serial.print(L);
  // Serial.print(",");
  // Serial.print(P);
  // Serial.print(",");
  // Serial.print(S);
  // Serial.print(",");
  // Serial.print(KalmanAngleRoll);
  // Serial.print(",");
  // Serial.print(KalmanAnglePitch);  
  // Serial.print(",");
  // Serial.print(AccZInertial);
  // Serial.print(",");
  // Serial.println(AltitudeBarometer);
  // Serial.println(",");
  Serial.print(AltitudeKalman);
  Serial.print(",");
  Serial.println(VelocityVerticalKalman);
  
  while (micros() - LoopTimer < 4000); 
  LoopTimer=micros();                  
}