#include "RobotPosition.h"
#include "PIDController.h"
#include "Robot.h"
#include "math.h"
#include "Command.h"
#include "Path.h"
#include "Wire.h"
#include <Arduino.h>

#define thresholdDistance 40
#define thresholdAngle 10

template <typename T> int sgn(T val) {
    return (T(0) < val) - (val < T(0));
}

double optimizeAngle(double a){
  double o = a;
  while(o > 180){
    o -= 360;
  }while(o < -180){
    o += 360;
  }
  return o;
}

void Robot::setMotors(int aSpeed, int bSpeed) {
  Serial.print(aSpeed);
  Serial.print(" ");
  Serial.print(bSpeed);
  Serial.print(" ");

  byte highByteA = aSpeed >> 8;
  byte lowByteA = aSpeed & 0xFF;
  byte highByteB = bSpeed >> 8;
  byte lowByteB = bSpeed & 0xFF;
  
  Wire.beginTransmission(9);

  int tempA = aDirection;
  int tempB = bDirection;

  if(tempA == -1){
    tempA = 0;
  }
  if(tempB == -1){
    tempB = 0;
  }
  Wire.write(1*(tempA)+2*(tempB)+4*(2));
  Wire.write(highByteA);
  Wire.write(lowByteA);
  Wire.write(highByteB);
  Wire.write(lowByteB);
  Wire.endTransmission();
}


Robot::Robot(){
  initialize(0, 0, 0, 0, 0);
}

Robot::Robot(int initialA, int initialB, double initialX, double initialY, double initialAngle){
  initialize(initialA, initialB, initialX, initialY, initialAngle);
}

void Robot::initialize(int initialA, int initialB, double initialX, double initialY, double initialAngle){
  position.set(initialA, initialB, initialX, initialY, initialAngle);
  distancePIDController.initialize(0, 1, 0, 0, position, position);
  anglePIDController.initialize(0, 0, 0, 0, initialAngle, initialAngle);
  aDirection = 1;
  bDirection = 1;
  running = false;
  initializeCommand();
}

void Robot::start(){
  running = true;
}

void Robot::updatePosition(double currentPositionA, double currentPositionB, double currentAngle){
  position.update(currentPositionA, currentPositionB, currentAngle);
}

void Robot::updateMotors(){
  anglePIDController.update(position.angle);
  distancePIDController.update(position);
  
  double speedAdjustment = anglePIDController.calculate();
  double netSpeed = distancePIDController.calculate();

  Serial.print(netSpeed);
  Serial.print(" ");

  if(speedAdjustment > 50){
    speedAdjustment = 50;
  }
  
  if(netSpeed > 50){
    netSpeed = 50;
  }

  if(netSpeed < 0){
    netSpeed = 0;
  }

  double aSpeed = netSpeed + speedAdjustment;
  double bSpeed = netSpeed - speedAdjustment;

  if(aSpeed<-255){
    aSpeed = -255;
  }
  if(bSpeed<-255){
    bSpeed = -255;
  }
  if(aSpeed>255){
    aSpeed = 255;
  }
  if(bSpeed>255){
    bSpeed = 255;
  }
  
  setMotors(aSpeed, bSpeed);
}

void Robot::initializeCommand(){
  Command* currentCommandPtr = path.getCurrentCommand();
  switch(currentCommandPtr -> type){
    case CommandType::Empty:
      break;
    case CommandType::Drive:
    {
      DriveCommand currentCommand = *((DriveCommand*) currentCommandPtr);
      distancePIDController.setTargetPosition(currentCommand.targetPosition);
      double targetAngle = atan2(currentCommand.targetPosition.y-position.y, currentCommand.targetPosition.x-position.x) * 180/M_PI;
      Serial.print(" ");
      double angle_diff = optimizeAngle(targetAngle - position.angle);
      targetAngle = position.angle + angle_diff;
      anglePIDController.setCoefficients(0, 0.24, 0, 0.25);
      anglePIDController.setTargetPosition(targetAngle);
      aDirection = 1;
      bDirection = 1;
    }
      break;
    case CommandType::Turn:
    {
      TurnCommand currentCommand = *((TurnCommand*) currentCommandPtr);
      anglePIDController.setCoefficients(15, 0.09, 0, 0.5);
      double targetAngle = currentCommand.angle;
      double angle_diff = optimizeAngle(targetAngle - position.angle);
      targetAngle = position.angle + angle_diff;
      // anglePIDController.setTargetPosition(targetAngle);
      anglePIDController.setTargetPosition(targetAngle);
      distancePIDController.setTargetPosition(position);
      // need to check if this is correct, may be reversed
      if(angle_diff>0){
        aDirection = 1;
        bDirection = -1;
      }else{
        aDirection = -1;
        bDirection = 1;
      }
    }
      break;
    case CommandType::Wait:
      break;
    case CommandType::Stall:
      break;
  }
}
void Robot::nextCommand(){
  // Serial.println("oh fuck");
  setMotors(0, 0);
  delay(2000);
  path.next();
  initializeCommand();
}

void Robot::update(double currentPositionA, double currentPositionB, double currentAngle){

  updatePosition(currentPositionA, currentPositionB, currentAngle);

  if(!running){
    setMotors(0, 0);
    return;
  }

  Command* currentCommandPtr = path.getCurrentCommand();

  // Serial.println(currentCommandPtr->type);

  switch(currentCommandPtr -> type){
    case CommandType::Empty:
    {
      nextCommand();
    }
      break;
    case CommandType::Drive:
    {
      DriveCommand currentCommand = *((DriveCommand*) currentCommandPtr);
      double targetAngle = atan2(currentCommand.targetPosition.y-position.y, currentCommand.targetPosition.x-position.x) * 180/M_PI;
      double angle_diff = optimizeAngle(targetAngle - position.angle);
      targetAngle = position.angle + angle_diff;
      Serial.print("a: ");
      Serial.print(targetAngle);
      Serial.print(" ");
      Serial.print(position.angle);
      Serial.print(" ");
       Serial.print(angle_diff);
      Serial.print(" ");
      anglePIDController.setTargetPosition(targetAngle);
      // Serial.print(position.x);
      // Serial.print(" ");
      // Serial.print(position.y);
      // Serial.print(" ");


      // Serial.println(currentCommand.targetPosition - position);
      if(currentCommand.targetPosition - position < thresholdDistance){
        nextCommand();
      }
    }
      break;
    case CommandType::Turn:
    {
      TurnCommand currentCommand = *((TurnCommand*) currentCommandPtr);
      double angle_diff = optimizeAngle(currentCommand.angle - position.angle);
      distancePIDController.setTargetPosition(position);
      Serial.print(angle_diff);
      Serial.print(" ");
      if(abs(angle_diff) < thresholdAngle){
        nextCommand();
      }
    }
      break;
    case CommandType::Wait:
    {
      WaitCommand currentCommand = *((WaitCommand*) currentCommandPtr);
      nextCommand();
    }
      break;
    case CommandType::Stall:
    {
      StallCommand currentCommand = *((StallCommand*) currentCommandPtr);
      nextCommand();
    }
      break;
  }

  // Serial.println(path.isFinished());

  if(path.isFinished()){
    running = false;
    setMotors(0, 0);
    return;
  }else{
    updateMotors();
  }

  return;
}