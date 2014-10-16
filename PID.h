/* 
 * File:   PID.h
 * Author: user
 *
 * Created on September 16, 2014, 9:07 AM
 */

#ifndef PID_H
#define	PID_H


//Constants used in some of the functions below
#define AUTOMATIC	1
#define MANUAL	0
#define DIRECT  0
#define REVERSE  1

struct PID
{
    float dispKp;              // * we'll hold on to the tuning parameters in user-entered
    float dispKi;              //   format for display purposes
    float dispKd;              //

    float kp;                  // * (P)roportional Tuning Parameter
    float ki;                  // * (I)ntegral Tuning Parameter
    float kd;                  // * (D)erivative Tuning Parameter

    long *myInput;              // * Pointers to the Input, Output, and Setpoint variables
    float *myOutput;             //   This creates a hard link between the variables and the
    long *mySetpoint;           //   PID, freeing the user from having to constantly tell us
                                  //   what these values are.  with pointers we'll just know.

    unsigned long lastTime;
    float ITerm, lastInput;

    unsigned long SampleTime;
    float outMin, outMax;

    bool controllerDirection;
    bool inAuto;    

    /*float error, output;
    float last_error;
    float Kpp, Kpd, Kpi, Kff;
    int output_max, deadband;
    */
};
//commonly used functions **************************************************************************
//PID(float*, float*, float*,      // * constructor.  links the PID to the Input, Output, and
//float, float, float, int);       //   Setpoint.  Initial tuning parameters are also set here
	
void InitializePID(int ID, long*, float*, long*, float, float, float, int, int, int);

void SetMode(int ID, int Mode);       // * sets PID to either Manual (0) or Auto (non-0)

bool ComputePID(int ID);                 // * performs the PID calculation.  it should be
                                      //   called every time loop() cycles. ON/OFF and
                                      //   calculation frequency can be set using SetMode
                                      //   SetSampleTime respectively

void SetOutputLimits(int ID, float, float); //clamps the output to a specific range. 0-255 by default, but
                                      //it's likely the user will want to change this depending on
                                      //the application

//available but not commonly used functions ********************************************************
void SetTunings(int ID, float, float, float);// * While most users will set the tunings once in the          	  //   constructor, this function gives the user the option
                                        //   of changing tunings during runtime for Adaptive control
void SetControllerDirection(int ID, int);       // * Sets the Direction, or "Action" of the controller. DIRECT
                                        //   means the output will increase when error is positive. REVERSE
                                        //   means the opposite.  it's very unlikely that this will be needed
                                        //   once it is set in the constructor.
void SetSampleTime(int ID, int);        // * sets the frequency, in Milliseconds, with which
                                        //   the PID calculation is performed.  default is 100

//Display functions ****************************************************************
float GetKp(int ID);         // These functions query the pid for interal values.
float GetKi(int ID);		//  they were created mainly for the pid front-end,
float GetKd(int ID);		// where it's important to know what is actually
int GetMode(int ID);		//  inside the PID.
int GetDirection(int ID);     //

void SetMotorDirection(int ID, int direction);
#endif	/* PID_H */

