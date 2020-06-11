#include <math.h> // (no semicolon)
#include <Wire.h>
#include <SoftwareSerial.h>
#include <DFPlayer_Mini_Mp3.h>
#include <Crc16.h>
Crc16 crc;

#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define NEOPIXEL_PIN 7

Adafruit_NeoPixel strip = Adafruit_NeoPixel(3, NEOPIXEL_PIN, NEO_GRBW + NEO_KHZ800);


/* MPU-6050 sensor */
#define MPU6050_ACCEL_XOUT_H 0x3B // R
#define MPU6050_PWR_MGMT_1 0x6B // R/W
#define MPU6050_PWR_MGMT_2 0x6C // R/W
#define MPU6050_WHO_AM_I 0x75 // R
#define MPU6050_I2C_ADDRESS 0x68

// ============================================================

#define SerialTxControl 6
#define RS485Transmit LOW
#define RS485Receive HIGH

int joystickPin[5] = {12,11,10,9,8};
int joyStickData[5] = {1,1,1,1,1};
int prevJoyStickData[5] = {1,1,1,1,1};
byte motorID[3] = {0x01,0x02,0x03};
SoftwareSerial RS485Serial(4,5); //RX, TX

// ============================================================

SoftwareSerial mp3Serial(A2, A3); // RX, TX


// ============================================================
/* Kalman filter */
struct GyroKalman{
  /* These variables represent our state matrix x */
  float x_angle, x_bias;

  /* Our error covariance matrix */
  float P_00, P_01, P_10, P_11;
  
  /*
  * Q is a 2x2 matrix of the covariance. Because we
  * assume the gyro and accelerometer noise to be independent
  * of each other, the covariances on the / diagonal are 0.
  * Covariance Q, the process noise, from the assumption
  * x = F x + B u + w
  * with w having a normal distribution with covariance Q.
  * (covariance = E[ (X - E[X])*(X - E[X])' ]
  * We assume is linear with dt
  */
  float Q_angle, Q_gyro;

  /*
  * Covariance R, our observation noise (from the accelerometer)
  * Also assumed to be linear with dt
  */
  float R_angle;
};

struct GyroKalman angX;
struct GyroKalman angY;
struct GyroKalman angZ;

/*
* R represents the measurement covariance noise. In this case,
* it is a 1x1 matrix that says that we expect 0.3 rad jitter
* from the accelerometer.
*/
static const float R_angle = 0.3;     //.3 default

/*
* Q is a 2x2 matrix that represents the process covariance noise.
* In this case, it indicates how much we trust the acceleromter
* relative to the gyros
*/
static const float Q_angle = 0.01;  //0.01 (Kalman)
static const float Q_gyro = 0.04; //0.04 (Kalman)

//These are the limits of the values I got out of the Nunchuk accelerometers (yours may vary).
const int lowX = -2150;
const int highX = 2210;
const int lowY = -2150;
const int highY = 2210;
const int lowZ = -2150;
const int highZ = 2550;

/* time */
unsigned long prevSensoredTime = 0;
unsigned long curSensoredTime = 0;

typedef union accel_t_gyro_union
{
  struct
  {
  uint8_t x_accel_h;
  uint8_t x_accel_l;
  uint8_t y_accel_h;
  uint8_t y_accel_l;
  uint8_t z_accel_h;
  uint8_t z_accel_l;
  uint8_t t_h;
  uint8_t t_l;
  uint8_t x_gyro_h;
  uint8_t x_gyro_l;
  uint8_t y_gyro_h;
  uint8_t y_gyro_l;
  uint8_t z_gyro_h;
  uint8_t z_gyro_l;
  } reg;

  struct
  {
    int x_accel;
    int y_accel;
    int z_accel;
    int temperature;
    int x_gyro;
    int y_gyro;
    int z_gyro;
  } value;
};

int xInit[5] = {0,0,0,0,0};
int yInit[5] = {0,0,0,0,0};
int zInit[5] = {0,0,0,0,0};
int initIndex = 0;
int initSize = 5;
int xCal = 0;
int yCal = 0;
int zCal = 1800;

int initializeIter = 0;
bool isBalenceCalib = false;
bool isBalenceStart = false;
int goalX=0;
int goalY=0;


int prevErrorX = 0;
int prevErrorY = 0;

void setup()
{
  Serial.begin(9600);
  mp3Serial.begin (9600);
  mp3_set_serial (mp3Serial); //set softwareSerial for DFPlayer-mini mp3 module
  delay(10);                     // delay 1ms to set volume
  mp3_set_volume(30);          // value 0~30
  delay(10);

  strip.begin();
  

  Serial.println("Program Starting...");
  strip.setPixelColor(0, strip.Color(255, 0, 0));
  strip.show();
  mp3_play(1);
  delay(3000);
  
  // ============== mp3 ===============

  RS485Serial.begin(9600);
  
  Serial.begin(9600);
  pinMode(SerialTxControl,OUTPUT);
  digitalWrite(SerialTxControl, RS485Transmit);
  delay(100);
  digitalWrite(SerialTxControl, RS485Receive);
  
  FrequencyPacket(motorID[0],0);
  FrequencyPacket(motorID[1],0);
  FrequencyPacket(motorID[2],0);

  Serial.println("Motor Initialize");
  strip.setPixelColor(1, strip.Color(255, 0, 0));
  strip.show();
  mp3_play(2);
  delay(3000);
  
  // =============== RS485 ================
  
  Wire.begin();
  
  int error;
  uint8_t c;

  initGyroKalman(&angX, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angY, Q_angle, Q_gyro, R_angle);
  initGyroKalman(&angZ, Q_angle, Q_gyro, R_angle);
  delay(100);

  // default at power-up:
  // Gyro at 250 degrees second
  // Acceleration at 2g
  // Clock source at internal 8MHz
  // The device is in sleep mode.
  //
  error = MPU6050_read (MPU6050_WHO_AM_I, &c, 1);
  Serial.print(F("WHO_AM_I : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // According to the datasheet, the 'sleep' bit
  // should read a '1'. But I read a '0'.
  // That bit has to be cleared, since the sensor
  // is in sleep mode at power-up. Even if the
  // bit reads '0'.
  error = MPU6050_read (MPU6050_PWR_MGMT_2, &c, 1);
  Serial.print(F("PWR_MGMT_2 : "));
  Serial.print(c,HEX);
  Serial.print(F(", error = "));
  Serial.println(error,DEC);

  // Clear the 'sleep' bit to start the sensor.
  MPU6050_write_reg (MPU6050_PWR_MGMT_1, 0);

  Serial.println("I M U Running");
  strip.setPixelColor(2, strip.Color(255, 0, 0));
  strip.show();
  mp3_play(3);
  delay(3000);
  
  // ============== IMU ===============

}


void loop()
{
  if(isBalenceStart){
    BalencingMode();  
  }
  else{
    MotorUsingJoystick();    
  }
}


void MotorUsingJoystick(){
    for(int i=0; i<5; i++){
      joyStickData[i] = digitalRead(joystickPin[i]);
    }

    // Start Button!
    if(joyStickData[0]==0){
      isBalenceStart=true;
      Serial.println("Calibration Start");
      for (int i = 0; i < strip.numPixels(); i++) {
        strip.setPixelColor(i, strip.Color(0, 0, 255));
      }
      strip.show();
      mp3_play(4);
    }

    // Case 1
    if(joyStickData[1]==0){
      FrequencyPacket(motorID[0],-3000); 
    }    
    else
    {
      FrequencyPacket(motorID[0],0); 
    }


    // Case 2
    if(joyStickData[2]==0){
      FrequencyPacket(motorID[1],-3000);
    }    
    else
    {
      FrequencyPacket(motorID[1],0);
    }

    // Case 3 앞
    if(joyStickData[3]==0){
    }    
    else
    {
    }

    // Case 4 뒤
    if(joyStickData[4]==0){
      FrequencyPacket(motorID[2],-3000); 
    }    
    else
    {
      FrequencyPacket(motorID[2],0);
    }


    for(int i=0; i<5; i++){
      prevJoyStickData[i] = joyStickData[i];
    }
}

void FrequencyPacket(byte _ID, int _frequency){
  int motorDir = _frequency>=0 ? 1:2;
  DirectionPacket(_ID,motorDir);

  int absFrequency = abs(_frequency);
  
  byte data[] = {_ID,0x06,0x00,0x04, absFrequency>>8,absFrequency};
  crc.clearCrc();
  int value = crc.Modbus(data,0,6);
  for(int i=0; i<6; i++){
    RS485Serial.write(data[i]);
  }
  RS485Serial.write((byte)(value));
  RS485Serial.write((byte)(value>>8));
  delay(20);
}

void DirectionPacket(byte _ID, int _command){
  byte data[] = {_ID,0x06,0x00,0x02,_command>>8, _command};
  crc.clearCrc();
  int value = crc.Modbus(data,0,6);
  for(int i=0; i<6; i++){
    RS485Serial.write(data[i]);
  }
  RS485Serial.write((byte)(value));
  RS485Serial.write((byte)(value>>8));
  delay(20);
}

void BalencingMode(){
  int error;
  double dT;
  accel_t_gyro_union accel_t_gyro;
  
  curSensoredTime = millis();
  
  error = MPU6050_read (MPU6050_ACCEL_XOUT_H, (uint8_t *) &accel_t_gyro, sizeof(accel_t_gyro));
  if(error != 0) {
    Serial.print(F("Read accel, temp and gyro, error = "));
    Serial.println(error,DEC);
  }
 
  uint8_t swap;
  #define SWAP(x,y) swap = x; x = y; y = swap
  SWAP (accel_t_gyro.reg.x_accel_h, accel_t_gyro.reg.x_accel_l);
  SWAP (accel_t_gyro.reg.y_accel_h, accel_t_gyro.reg.y_accel_l);
  SWAP (accel_t_gyro.reg.z_accel_h, accel_t_gyro.reg.z_accel_l);
  SWAP (accel_t_gyro.reg.t_h, accel_t_gyro.reg.t_l);
  SWAP (accel_t_gyro.reg.x_gyro_h, accel_t_gyro.reg.x_gyro_l);
  SWAP (accel_t_gyro.reg.y_gyro_h, accel_t_gyro.reg.y_gyro_l);
  SWAP (accel_t_gyro.reg.z_gyro_h, accel_t_gyro.reg.z_gyro_l);
  
  if(prevSensoredTime > 0) {
    int gx1=0, gy1=0, gz1 = 0;
    float gx2=0, gy2=0, gz2 = 0;

    int loopTime = curSensoredTime - prevSensoredTime;

    gx2 = angleInDegrees(lowX, highX, accel_t_gyro.value.x_gyro);
    gy2 = angleInDegrees(lowY, highY, accel_t_gyro.value.y_gyro);
    gz2 = angleInDegrees(lowZ, highZ, accel_t_gyro.value.z_gyro);

    predict(&angX, gx2, loopTime);
    predict(&angY, gy2, loopTime);
    predict(&angZ, gz2, loopTime);

    gx1 = update(&angX, accel_t_gyro.value.x_accel) / 10;
    gy1 = update(&angY, accel_t_gyro.value.y_accel) / 10;
    gz1 = update(&angZ, accel_t_gyro.value.z_accel) / 10;
   
    if(initIndex < initSize) {
      xInit[initIndex] = gx1;
      yInit[initIndex] = gy1;
      zInit[initIndex] = gz1;
      if(initIndex == initSize - 1) {
        int sumX = 0; int sumY = 0; int sumZ = 0;
        for(int k=1; k <= initSize; k++) {
          sumX += xInit[k];
          sumY += yInit[k];
          sumZ += zInit[k];
        }

        xCal -= sumX/(initSize -1);
        yCal -= sumY/(initSize -1);
        zCal = (sumZ/(initSize -1) - zCal);
      }
      initIndex++;
    }
    else {
        gx1 += xCal;
        gy1 += yCal;
    }

//    Serial.print(gx1, DEC);
//    Serial.print(F(", "));
//    Serial.println(gy1, DEC);

    if(initializeIter>20){
      if(!isBalenceCalib){
        Serial.println("Calibration Completed, Balencing Mode On");
        for (int i = 0; i < strip.numPixels(); i++) {
          strip.setPixelColor(i, strip.Color(0, 255, 0));
        }
        strip.show();
        mp3_play(5);
        
        isBalenceCalib = true;
        goalX = gx1;
        goalY = gy1;
      }
      int errorX = goalX - gx1;
      int errorY = goalY - gy1;
      BalencingMotor(errorY,-errorX);
    }
    else{
      initializeIter++;
      delay(200); 
    }
  }

  prevSensoredTime = curSensoredTime;
}

float P_controlX = 0;
float I_controlX = 0;
float D_controlX = 0;

float P_controlY = 0;
float I_controlY = 0;
float D_controlY = 0;

float prevPIDTime =0;

float K_P = 15;
float K_I = 0.001;
float K_D = 1;

float scale = 2.0;
void BalencingMotor(int _errorX, int _errorY){
  float nowPIDTime = millis();
  float loopPIDTime = nowPIDTime - prevPIDTime;
  
  P_controlX = K_P * _errorX;
  I_controlX += K_I * _errorX * loopPIDTime;
  D_controlX = K_D * (_errorX - prevErrorX) / loopPIDTime;

  P_controlY = K_P * _errorY;
  I_controlY += K_I * _errorY * loopPIDTime;
  D_controlY = K_D * (_errorY - prevErrorY) / loopPIDTime;

  float PID_ControlX = (P_controlX + I_controlX + D_controlX)/scale;
  float PID_ControlY = (P_controlY + I_controlY + D_controlY)/scale;

  float MotorFrequency1 = PID_ControlY+PID_ControlX;
  float MotorFrequency2 = PID_ControlY-PID_ControlX;
  float MotorFrequency3 = -PID_ControlY;

  if(abs(MotorFrequency1)>6000) MotorFrequency1 = 6000*(MotorFrequency1/abs(MotorFrequency1));
  if(abs(MotorFrequency2)>6000) MotorFrequency2 = 6000*(MotorFrequency2/abs(MotorFrequency2));
  if(abs(MotorFrequency3)>6000) MotorFrequency3 = 6000*(MotorFrequency3/abs(MotorFrequency3));
  
  Serial.print(" M1: ");
  Serial.print(MotorFrequency1);
  Serial.print(" M2: ");
  Serial.print(MotorFrequency2);
  Serial.print(" M3: ");
  Serial.println(MotorFrequency3);
  // MotorCode

  FrequencyPacket(motorID[0],(int)MotorFrequency1);
  FrequencyPacket(motorID[1],(int)MotorFrequency2);
  FrequencyPacket(motorID[2],(int)MotorFrequency3);
  
  prevErrorX = _errorX;
  prevErrorY = _errorY;
  prevPIDTime = nowPIDTime;
}

int MPU6050_read(int start, uint8_t *buffer, int size)
{
  int i, n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start);
  if (n != 1)
    return (-10);
  
  n = Wire.endTransmission(false); // hold the I2C-bus
  if (n != 0)
    return (n);
  
  // Third parameter is true: relase I2C-bus after data is read.
  Wire.requestFrom(MPU6050_I2C_ADDRESS, size, true);
  i = 0;
  while(Wire.available() && i<size)
  {
    buffer[i++]=Wire.read();
  }
  if ( i != size)
    return (-11);
  return (0); // return : no error
}

int MPU6050_write(int start, const uint8_t *pData, int size)
{
  int n, error;
  
  Wire.beginTransmission(MPU6050_I2C_ADDRESS);
  
  n = Wire.write(start); // write the start address
  if (n != 1)
    return (-20);
    
  n = Wire.write(pData, size); // write data bytes
  if (n != size)
    return (-21);
    
  error = Wire.endTransmission(true); // release the I2C-bus
  if (error != 0)
    return (error);
  return (0); // return : no error
}

int MPU6050_write_reg(int reg, uint8_t data)
{
  int error;
  error = MPU6050_write(reg, &data, 1);
  return (error);
}

float angleInDegrees(int lo, int hi, int measured) {
  float x = (hi - lo)/180.0;
  return (float)measured/x;
}

void initGyroKalman(struct GyroKalman *kalman, const float Q_angle, const float Q_gyro, const float R_angle) {
  kalman->Q_angle = Q_angle;
  kalman->Q_gyro = Q_gyro;
  kalman->R_angle = R_angle;
  
  kalman->P_00 = 0;
  kalman->P_01 = 0;
  kalman->P_10 = 0;
  kalman->P_11 = 0;
}

/*
* The kalman predict method.
* kalman    the kalman data structure
* dotAngle    Derivitive Of The (D O T) Angle. This is the change in the angle from the gyro.
*           This is the value from the Wii MotionPlus, scaled to fast/slow.
* dt        the change in time, in seconds; in other words the amount of time it took to sweep dotAngle
*/
void predict(struct GyroKalman *kalman, float dotAngle, float dt) {
  kalman->x_angle += dt * (dotAngle - kalman->x_bias);
  kalman->P_00 += -1 * dt * (kalman->P_10 + kalman->P_01) + dt*dt * kalman->P_11 + kalman->Q_angle;
  kalman->P_01 += -1 * dt * kalman->P_11;
  kalman->P_10 += -1 * dt * kalman->P_11;
  kalman->P_11 += kalman->Q_gyro;
}

/*
* The kalman update method
* kalman  the kalman data structure
* angle_m   the angle observed from the Wii Nunchuk accelerometer, in radians
*/
float update(struct GyroKalman *kalman, float angle_m) {
  const float y = angle_m - kalman->x_angle;
  const float S = kalman->P_00 + kalman->R_angle;
  const float K_0 = kalman->P_00 / S;
  const float K_1 = kalman->P_10 / S;
  
  kalman->x_angle += K_0 * y;
  kalman->x_bias += K_1 * y;
  kalman->P_00 -= K_0 * kalman->P_00;
  kalman->P_01 -= K_0 * kalman->P_01;
  kalman->P_10 -= K_1 * kalman->P_00;
  kalman->P_11 -= K_1 * kalman->P_01;
  return kalman->x_angle;
}
