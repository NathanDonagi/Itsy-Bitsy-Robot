#ifndef RobotPosition_h
#define RobotPosition_h

#include <Arduino.h>

class RobotPosition {
  public:
    double x, y, angle;
    int previousPositionA, previousPositionB;
    RobotPosition();
    RobotPosition(double x_, double y_);
    RobotPosition(double initialX, double initialY, double initialAngle);
    RobotPosition(int initialA, int initialB, double initialX, double initialY, double initialAngle);
    void set(int initialA, int initialB, double initialX, double initialY, double initialAngle);
    void update(int currentPositionA, int currentPositionB, double currentAngle);
    double linearDistance(RobotPosition o);
    double angularDistance(RobotPosition o);
    double operator-(RobotPosition o);
};

#endif