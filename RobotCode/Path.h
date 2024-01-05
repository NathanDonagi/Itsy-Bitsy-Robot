#ifndef Path_h
#define Path_h

#include <Arduino.h>
#include "Command.h"

class Path {
  public:
    DriveCommand driveCommands[32];
    TurnCommand turnCommands[32];
    Command* commandList[128];
    int length, commandIndex;
    Path();
    bool isFinished();
    void next();
    Command* getCurrentCommand();
};

#endif
