#ifndef PIDController_h
#define PIDController_h

#include <Arduino.h>

template <typename T> class PIDController {
  private:
    T targetPosition, currentPosition;
    double currentTime, derivitive, integral, bias, kp, ki, kd;
  public:
    PIDController();
    PIDController(double bias_, double kp_, double ki_, double kd_, T currentPosition_, T targetPosition_);
    void initialize(double bias_, double kp_, double ki_, double kd_, T currentPosition_, T targetPosition_);
    void setCoefficients(double bias_, double kp_, double ki_, double kd_);
    void setTargetPosition(T newTargetPosition);
    void update(T newPosition);
    double calculate();
};

#include "PIDController.tpp"

#endif