#include "MPU6050_6Axis_MotionApps20.h"
#include "math.h"
#include "Robot.h"
#include "Wire.h"

#define START_PIN 3
#define LED_PIN 13

// MPU control/status vars
MPU6050 mpu;
bool dmpReady = false;   // set true if DMP init was successful
uint8_t mpuIntStatus;    // holds actual interrupt status byte from MPU
uint8_t devStatus;       // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;     // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;      // count of all bytes currently in FIFO
uint8_t fifoBuffer[64];  // FIFO storage buffer

// orientation/motion vars
Quaternion q;         // [w, x, y, z]         quaternion container
VectorFloat gravity;  // [x, y, z]            gravity vector
float ypr[3];         // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

Robot robot;

float currentAngle;

volatile bool started;
int aPosition, bPosition;

void readEncoders(){
  Wire.requestFrom(9, 4);
  byte highByteA = Wire.read();
  byte lowByteA = Wire.read();
  byte highByteB = Wire.read();
  byte lowByteB = Wire.read();

  aPosition = (highByteA<<8)+lowByteA;
  bPosition = (highByteB<<8)+lowByteB;
}

void buttonPressed(){
  started = true;
}

void setup() {

  started = false;
  aPosition = 0;
  bPosition = 0;

  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(9600);
 
  while (!Serial)

  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(START_PIN, INPUT);
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
 
    mpu.setXGyroOffset(131);
    mpu.setYGyroOffset(-95);
    mpu.setZGyroOffset(-27);
    mpu.setXAccelOffset(-3974);
    mpu.setYAccelOffset(2425);
    mpu.setZAccelOffset(1462);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    // mpu.CalibrateAccel(20);
    // mpu.CalibrateGyro(20);
    mpu.PrintActiveOffsets();

    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(START_PIN), buttonPressed, RISING);

    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  readEncoders();
  robot.initialize(aPosition, bPosition, 0, 0, 0);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, 13);
}

void loop() {
  if(!dmpReady) return;

  if(started && !robot.running){
    robot.start();
    delay(1000);
    started = false;
  }

  readEncoders();
  if (dmpReady && mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    currentAngle =  (ypr[0] * 180.0 / M_PI);
  }

  robot.update(aPosition, bPosition, currentAngle);
  
  // Serial.print(aPosition);
  // Serial.print(" ");
  // Serial.print(bPosition);
  // Serial.print(" ");
  // Serial.print(currentAngle);
  Serial.println("");
}