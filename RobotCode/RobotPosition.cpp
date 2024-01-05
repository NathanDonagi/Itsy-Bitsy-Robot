#include "RobotPosition.h"
#include "math.h"
#include <Arduino.h>

#define distanceScale 0.90

RobotPosition::RobotPosition(){
  set(0, 0, 0, 0, 0);
}

RobotPosition::RobotPosition(double initialX, double initialY, double initialAngle){
  set(0, 0, initialX, initialY, initialAngle);
}

RobotPosition::RobotPosition(double x_, double y_){
  set(0, 0, x, y, 0);
}

RobotPosition::RobotPosition(int initialA, int initialB, double initialX, double initialY, double initialAngle){
  set(initialA, initialB, initialX, initialY, initialAngle);
}

void RobotPosition::set(int initialA, int initialB, double initialX, double initialY, double initialAngle){
  x = initialX;
  y = initialY;
  angle = initialAngle;
  previousPositionA = initialA;
  previousPositionB = initialB;
}

void RobotPosition::update(int currentPositionA, int currentPositionB, double currentAngle){
  double wheelAChange = (double) (currentPositionA - previousPositionA);
  double wheelBChange = (double) (currentPositionB - previousPositionB);
  previousPositionA = currentPositionA;
  previousPositionB = currentPositionB;
  angle = currentAngle;
  x += (wheelAChange+wheelBChange) * cos(angle * M_PI/180);
  y += (wheelAChange+wheelBChange) * sin(angle * M_PI/180);
}

double RobotPosition::linearDistance(RobotPosition o){
  return sqrt((x-o.x)*(x-o.x)+(y-o.y)*(y-o.y));
}

double RobotPosition::angularDistance(RobotPosition o){
  return abs(angle-o.angle);
}

double RobotPosition::operator-(RobotPosition o){
  return linearDistance(o);
}