#ifndef Command_h
#define Command_h

#include <Arduino.h>
#include "RobotPosition.h"

enum CommandType {Empty, Drive, Turn, Wait, Stall};

struct Command {
  CommandType type;
  Command(){
    type = CommandType::Empty;
  }
};

struct DriveCommand : public Command{
  RobotPosition targetPosition;
  DriveCommand(RobotPosition targetPosition_){
    type = CommandType::Drive;
    targetPosition = targetPosition_;
  }
  DriveCommand(){
    type = CommandType::Drive;
    targetPosition = RobotPosition();
  }
};

struct TurnCommand : public Command{
  double angle;
  TurnCommand(double angle_){
    type = CommandType::Turn;
    angle = angle_;
  }
  TurnCommand(){
    type = CommandType::Turn;
    angle = 0;
  }
};

struct WaitCommand : public Command{
  unsigned long time;
  WaitCommand(unsigned long time_){
    type = CommandType::Wait;
    time = time_;
  }
  WaitCommand(){
    type = CommandType::Wait;
    time = 0;
  }
};

struct StallCommand : public Command{
  unsigned long time;
  StallCommand(unsigned long time_){
    type = CommandType::Stall;
    time = time_;
  }
  StallCommand(){
    type = CommandType::Stall;
    time = 0;
  }
};

#endif