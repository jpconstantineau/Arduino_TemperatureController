#include <arduino.h>
#include "sensors.h"
#ifndef CONTROL_H_
#define CONTROL_H_

#include "PID_v1.h"            // PID Library
#ifdef TUNE
  #include <PID_AutoTune_v0.h>   // PID Autotune Library 
#endif
enum operatingState { oSTART = 0, oOFF, oRUN, oAUTO, oMAN};

// ---------- CONTROL STRUCTURE ---------------

struct PIDStruct {
  DeviceAddress deviceAddress;
  double Input;
  double Setpoint;
  double Output;
  double Kp;
  double Ki;
  double Kd;
  unsigned long WindowSize;
  int SampleTime;
  char version_of_program[4];
}; 

extern PIDStruct default_PID_settings;

extern PIDStruct *PIDData[NUMBER_OF_PID];

// ------PID Variables------------------------------------------------

extern double Setpoint; 

extern double Input, Ambient, Element;
extern double Output ;

// pid tuning parameters
//extern double Kp;
//extern double Ki;
//extern double Kd;
//Specify the links and initial tuning parameters

//extern PID myPID;
#ifdef TUNE
// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;

double aTuneStep=500;
double aTuneNoise=1;
unsigned int aTuneLookBack=20;
PID_ATune aTune(&Input, &Output);
#endif
extern boolean tuning[NUMBER_OF_PID];
extern operatingState opState[NUMBER_OF_PID];


extern unsigned long WindowSize[NUMBER_OF_PID] ; 
extern unsigned long windowStartTime[NUMBER_OF_PID];
extern unsigned long onTime[NUMBER_OF_PID] ;
extern boolean      Output_Status[NUMBER_OF_PID] ;

void SetupPIDs();
void RunPID();
void Start(int PIDid) ;
void Off(int PIDid);
void Run(int PIDid);
void Auto(int PIDid);
void Man (int PIDid);
void DoControl(int PIDid);
void StartAutoTune(int PIDid);
void FinishAutoTune(int PIDid);

#endif
