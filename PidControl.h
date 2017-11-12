/**
 * PidControl.h
 * 
 * Created On: 9/28/2017
 * Last Modified: 11/12/2017
 * Author: Leif Sahyun
 * 
 * Handles PID control for a motor and an encoder.
 * This class keeps track of error values and has public variables for the constants kp, ki, and kd.
 * Calling update on a PidControl object will automatically update the value written to the controlled motor
 */
#include "Arduino.h"

class PidControl{
  public:
    PidControl(long *input, int *output);
    PidControl(long *input, int *output, long goal);
    void setGoal(long goal);
    int getError();
    bool isInTolerance(int errorTolerance, double errorDerivTolerance);
    bool isInTolerance();
    void updatePid();

    //PID constants are public member variables so they can be modified by the main code easily
    double kp=300;
    double ki=0.01;
    double kd=30;
    //Other public member variables include the scale, defaultTolerance, and setpoint
    int scale = 4000;
    int defaultTolerance = 10;
    //If toPrint is true, PidControl will print a summary of its control each time updatePid is called to aid debugging
    bool toPrint = false;
    
  private:
    long setpoint = 0;
    long *processVariable;
    int *out;
    int lastError;
    long sumError;
    double derivError;
    unsigned long lastTime;
};
