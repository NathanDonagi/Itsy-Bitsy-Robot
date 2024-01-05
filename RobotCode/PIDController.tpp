#include <Arduino.h>

template <class T>
PIDController<T>::PIDController(double bias_, double kp_, double ki_, double kd_, T currentPosition_, T targetPosition_){
  initialize(bias_, kp_, ki_, kd_, currentPosition_, targetPosition_);
  return;
}

template <class T>
PIDController<T>::PIDController(){
  initialize(0, 0, 0, 0, T(), T());
  return;
}

template <class T>
void PIDController<T>::initialize(double bias_, double kp_, double ki_, double kd_, T currentPosition_, T targetPosition_){
  currentTime = millis();
  currentPosition = currentPosition_;
  targetPosition = targetPosition_;
  currentTime = 0;
  derivitive = 0;
  integral = 0;
  bias = bias_;
  kp = kp_;
  ki = ki_;
  kd = kd_;
  return;
}

template <class T>
void PIDController<T>::setCoefficients(double bias_, double kp_, double ki_, double kd_){
  bias = bias_;
  kp = kp_;
  ki = ki_;
  kd = kd_;
  return;
}


template <class T>
void PIDController<T>::setTargetPosition(T newTargetPosition){
  update(currentPosition);
  targetPosition = newTargetPosition;
  return;
}

template <class T>
void PIDController<T>::update(T newPosition){
  unsigned long deviceTime = millis();
  derivitive = newPosition-currentPosition;
  integral += (targetPosition-newPosition)*(deviceTime-currentTime);
  currentPosition = newPosition;
  currentTime = deviceTime;
  return;
};

template <class T>
double PIDController<T>::calculate(){
  double out = (targetPosition-currentPosition)*kp + integral*ki - derivitive*kd; 
  return bias*sgn(out) + out;
};