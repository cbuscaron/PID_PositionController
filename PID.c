

#include <p33Exxxx.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "user.h"
#include "PID.h"

struct PID PID[6];

void InitializePID(int ID, long* Input, float* Output, long* Setpoint,
        float Kp, float Ki, float Kd, int ControllerDirection, int PWMP, int PWMN) {
    // From the constuctor function in the arduino PID library
    PID[ID].myOutput = Output;
    PID[ID].myInput = Input;
    PID[ID].mySetpoint = Setpoint;
    PID[ID].inAuto = true;
    SetOutputLimits(ID, PWMN, PWMP); // Set the default output range to -255-255
    SetControllerDirection(ID, ControllerDirection);
    SetTunings(ID, Kp, Ki, Kd);
    PID[ID].SampleTime = 100;
    PID[ID].lastTime = millis() - PID[ID].SampleTime; 

    // From arduino Initalize() function 
    PID[ID].ITerm = *PID[ID].myOutput;
    PID[ID].lastInput = *PID[ID].myInput;
    if (PID[ID].ITerm > PID[ID].outMax) PID[ID].ITerm = PID[ID].outMax;
    else if (PID[ID].ITerm < PID[ID].outMin) PID[ID].ITerm = PID[ID].outMin;
}

bool ComputePID(int ID) {

    if (!PID[ID].inAuto) return false;
    unsigned long now = millis();
    unsigned long timeChange = (now - PID[ID].lastTime);
    if (timeChange >= PID[ID].SampleTime) {
        /*Compute all the working error variables*/
        float input = *PID[ID].myInput;
        float error = *PID[ID].mySetpoint - input;
        PID[ID].ITerm += (PID[ID].ki * error);
        if (PID[ID].ITerm > PID[ID].outMax) PID[ID].ITerm = PID[ID].outMax;
        else if (PID[ID].ITerm < PID[ID].outMin) PID[ID].ITerm = PID[ID].outMin;
        float dInput = (error - PID[ID].lastInput);

        /*Compute PID Output*/
        float output = 0;
        output = (PID[ID].kp * error) + PID[ID].ITerm + (PID[ID].kd * dInput);
        if (output > PID[ID].outMax) output = PID[ID].outMax;
        else if (output < PID[ID].outMin) output = PID[ID].outMin;        
        if (output > 0) {
            *PID[ID].myOutput = output;
            SetMotorDirection(ID, 1);
        } else {
            *PID[ID].myOutput = -output;
            SetMotorDirection(ID, 0);
        }

        /*Remember some variables for next time*/
        PID[ID].lastInput = error;
        PID[ID].lastTime = now;
        return true;
    } else return false;
}

void SetTunings(int ID, float Kp, float Ki, float Kd) {

    if (Kp < 0 || Ki < 0 || Kd < 0) return;

    PID[ID].dispKp = Kp;
    PID[ID].dispKi = Ki;
    PID[ID].dispKd = Kd;

    float SampleTimeInSec = ((float) PID[ID].SampleTime) / 1000;
    PID[ID].kp = Kp;
    PID[ID].ki = Ki * SampleTimeInSec;
    if (Kd == 0) PID[ID].kd = 0;
    else PID[ID].kd = Kd / SampleTimeInSec;    

    if (PID[ID].controllerDirection == REVERSE) {
        PID[ID].kp = (0 - PID[ID].kp);
        PID[ID].ki = (0 - PID[ID].ki);
        PID[ID].kd = (0 - PID[ID].kd);
    }
}

void SetControllerDirection(int ID, int direction) {
    if (PID[ID].inAuto && direction != PID[ID].controllerDirection) {
        PID[ID].kp = (0 - PID[ID].kp);
        PID[ID].ki = (0 - PID[ID].ki);
        PID[ID].kd = (0 - PID[ID].kd);
    }
    PID[ID].controllerDirection = direction;
}

void SetSampleTime(int ID, int NewSampleTime) {
    if (NewSampleTime > 0) {
        float ratio = (float) NewSampleTime / (float) PID[ID].SampleTime;
        PID[ID].ki *= ratio;
        PID[ID].kd /= ratio;
        PID[ID].SampleTime = (unsigned long) NewSampleTime;
    }
}

void SetOutputLimits(int ID, float Min, float Max) {
    if (Min >= Max) return;
    PID[ID].outMin = Min;
    PID[ID].outMax = Max;

    if (PID[ID].inAuto) {
        if (*PID[ID].myOutput > PID[ID].outMax) *PID[ID].myOutput = PID[ID].outMax;
        else if (*PID[ID].myOutput < PID[ID].outMin) *PID[ID].myOutput = PID[ID].outMin;

        if (PID[ID].ITerm > PID[ID].outMax) PID[ID].ITerm = PID[ID].outMax;
        else if (PID[ID].ITerm < PID[ID].outMin) PID[ID].ITerm = PID[ID].outMin;
    }
}

float GetKp(int ID) {
    return PID[ID].dispKp;
}

float GetKi(int ID) {
    return PID[ID].dispKi;
}

float GetKd(int ID) {
    return PID[ID].dispKd;
}

int GetMode(int ID) {
    return PID[ID].inAuto ? AUTOMATIC : MANUAL;
}

int GetDirection(int ID) {
    return PID[ID].controllerDirection;
}

void SetMotorDirection(int ID, int direction) {
    switch (ID) {
        case 1:
             if (direction) {
                 MOTOR2_INA = 0;
                 MOTOR2_INB = 1;
             } else {
                 MOTOR2_INA = 1;
                 MOTOR2_INB = 0;
             }
            break;            
        case 2:
             if (direction) {
                 MOTOR3_INA = 1;
                 MOTOR3_INB = 0;
             } else {
                 MOTOR3_INA = 0;
                 MOTOR3_INB = 1;
             }
            break;            
        case 3:
             if (direction) {
                 MOTOR4_INA = 1;
                 MOTOR4_INB = 0;
             } else {                  
                 MOTOR4_INA = 0;
                 MOTOR4_INB = 1;
             }
            break;
        default:
            break; 
    }
}
