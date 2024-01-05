#include "Path.h"
#include "Command.h"
#include <Arduino.h>

Path::Path(){
  commandIndex = 0;
  length = 15;
  //approx 0.25 cm per unit
  driveCommands[0] = DriveCommand(RobotPosition(100, 0, 0));
  driveCommands[1] = DriveCommand(RobotPosition(100, 200, 0));
  driveCommands[2] = DriveCommand(RobotPosition(300, 200, 0));
  driveCommands[3] = DriveCommand(RobotPosition(300, 0, 0));
  driveCommands[4] = DriveCommand(RobotPosition(500, 0, 0));
  driveCommands[5] = DriveCommand(RobotPosition(500, -200, 0));
  driveCommands[6] = DriveCommand(RobotPosition(500, 200, 0));
  driveCommands[7] = DriveCommand(RobotPosition(700, 200, 0));


  turnCommands[0] = TurnCommand(90);
  turnCommands[1] = TurnCommand(0);
  turnCommands[2] = TurnCommand(270);
  turnCommands[3] = TurnCommand(0);
  turnCommands[4] = TurnCommand(270);
  turnCommands[5] = TurnCommand(90);
  turnCommands[6] = TurnCommand(0);
  
  commandList[0] = &driveCommands[0];
  commandList[1] = &turnCommands[0];
  commandList[2] = &driveCommands[1];
  commandList[3] = &turnCommands[1];
  commandList[4] = &driveCommands[2];
  commandList[5] = &turnCommands[2];
  commandList[6] = &driveCommands[3];
  commandList[7] = &turnCommands[3];
  commandList[8] = &driveCommands[4];
  commandList[9] = &turnCommands[4];
  commandList[10] = &driveCommands[5];
  commandList[11] = &turnCommands[5];
  commandList[12] = &driveCommands[6];
  commandList[13] = &turnCommands[6];
  commandList[14] = &driveCommands[7];

}
bool Path::isFinished(){
  return commandIndex >= length;
}

Command* Path::getCurrentCommand(){
    return commandList[commandIndex];
}

void Path::next(){
  commandIndex += 1;
}