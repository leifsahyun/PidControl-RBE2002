/**
 * PidControl.cpp
 * 
 * Created On: 9/28/2017
 * Last Modified: 11/12/2017
 * Author: Leif Sahyun
 */

#include "PidControl.h"

/**
 * Constructors initialize the PidControl object with pointers to the motor and encoder as well as an optional initial goal.
 */
PidControl::PidControl(long *input, int *output, long goal){
  processVariable = input;
  out = output;
  setpoint = goal;
  lastTime = micros();
}

PidControl::PidControl(long *input, int *output){
  PidControl::PidControl(input, output, 0);
}


/**
 * Sets the value of setpoint and clears sumError
 */
void PidControl::setGoal(long goal){
  setpoint = goal;
  sumError = 0;
}

/**
 * Returns current error
 */
int PidControl::getError(){
  noInterrupts();
  int err = *processVariable-setpoint;
  interrupts();
  return err;
}

/**
 * Returns true if the current position of the mechanism is within an acceptable tolerance of the setpoint
 * The errorDerivTolerance ensures the mechanism is not moving too fast past the setpoint to reduce overshoot
 */
bool PidControl::isInTolerance(int errorTolerance, double errorDerivTolerance){
  int err = getError();
  return ((err>-errorTolerance)&&(err<errorTolerance)
    &&(derivError>-errorDerivTolerance)&&(derivError<errorDerivTolerance));
}

/**
 * The polymorphic version of isInTolerance with no inputs returns true if the error is within the object's default tolerance
 */
bool PidControl::isInTolerance(){
  int err = getError();
  return (err>-defaultTolerance)&&(err<defaultTolerance);
}

/**
 * Updates the PID control.
 * This is not done on a specific interval so the time difference since the last update is measured and used in the integral and derivative calculations
 * If toPrint is set to true, updatePid will print a summary of the current PidControl to the Serial output for debugging
 */
void PidControl::updatePid(){
  unsigned long now = micros();
  int deltaTime = now - lastTime; 
  int error = getError(); 
  int deltaError = error - lastError; 
  sumError += (error * deltaTime); //Integral(error) = Sum(error*deltaTime) for a given time interval as deltaTime approaches 0
  derivError = deltaError/deltaTime; //Derivative(error) = deltaError/deltaTime;
  double motorValue = kp*error + ki*sumError + kd*derivError; //Calculation of motorValue with PID constants
  motorValue = motorValue/scale+90; //The motorValue yielded above is a positive or negative number that may be large depending on the number of encoder ticks in the mechanism's range of motion. Dividing motorValue by a constant scale and adding 90 yields values between 0 and 180
  motorValue = constrain(motorValue, 0, 180); //In case the previous line did not fully scale the motorValue to the [0, 180] range, this constrains it to be only within that range.
  if(toPrint){ //If toPrint is true, print a summary of the PID control for debugging
    Serial.print("position: ");
    Serial.println(setpoint+error);
    Serial.print("error: ");
    Serial.println(error);
    Serial.print("output: ");
    Serial.println(motorValue);
  }
  *out = motorValue; //Otherwise write the motor value to the regular Servo object
  lastTime = now;
  lastError = error;
}

