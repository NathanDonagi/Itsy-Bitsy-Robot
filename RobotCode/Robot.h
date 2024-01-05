#ifndef Robot_h
#define Robot_h

#include "PIDController.h"
#include "RobotPosition.h"
#include "Path.h"
#include "Command.h"
#include <Arduino.h>
#include "Wire.h"

class Robot {
  private:
    Path path;
    RobotPosition position;
    PIDController<RobotPosition> distancePIDController;
    PIDController<double> anglePIDController;
    int aDirection, bDirection;
  public:
    bool running;
    Robot();
    Robot(int initialA, int initialB, double initialX, double initialY, double initialAngle);
    void initialize(int initialA, int initialB, double initialX, double initialY, double initialAngle);
    void updatePosition(double currentPositionA, double currentPositionB, double currentAngle);
    void updateMotors();
    void update(double currentPositionA, double currentPositionB, double currentAngle);
    void nextCommand();
    void setMotors(int a, int b);
    void start();
    void setPath(Path path_);
    void initializeCommand();
};

#endif