
#include "display.h"
#include "control.h"
#include "io.h"

// ------PID Variables------------------------------------------------



double  Ambient, Element;


PID *PIDs[NUMBER_OF_PID];
PIDStruct *PIDData[NUMBER_OF_PID];

boolean tuning[NUMBER_OF_PID];
operatingState opState[NUMBER_OF_PID];


unsigned long WindowSize[NUMBER_OF_PID]; 
unsigned long windowStartTime[NUMBER_OF_PID];
unsigned long onTime[NUMBER_OF_PID] ;
boolean      Output_Status[NUMBER_OF_PID] ;

PIDStruct default_PID_settings = {
  // The default values
  {0,0,0,0,0,0,0,0},
  25.5, 25.0, 0.0,
  850.0, 10.0, 1.0,
  100000, 1000,
  CONFIG_VERSION
};


void SetupPIDs()
{
   for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
    {
      PIDData[PIDid] =new PIDStruct;
      LoadPIDStruct(PIDid,PIDData[PIDid]);
      Serial.print("address:"); serialprintAddress(PIDData[PIDid]->deviceAddress);Serial.println("");
      Serial.print("SP:"); Serial.println(PIDData[PIDid]->Setpoint);
      
      PIDs[PIDid] = new PID(&PIDData[PIDid]->Input, &PIDData[PIDid]->Output, &PIDData[PIDid]->Setpoint, PIDData[PIDid]->Kp, PIDData[PIDid]->Ki, PIDData[PIDid]->Kd, DIRECT);
      PIDs[PIDid]->SetTunings(PIDData[PIDid]->Kp,PIDData[PIDid]->Ki,PIDData[PIDid]->Kd);
      PIDs[PIDid]->SetSampleTime(PIDData[PIDid]->SampleTime);
      PIDs[PIDid]->SetOutputLimits(0, PIDData[PIDid]->WindowSize);
      opState[PIDid] = oSTART;
      WindowSize[PIDid] = PIDData[PIDid]->WindowSize; // 100 second Time Proportional Output window
      onTime[PIDid] = 50000;
      Output_Status[PIDid] = false;
//      Output[PIDid] =0;
      tuning[PIDid] = false;
    }
}

void RunPID()
{
   for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
   {
     switch (opState[PIDid])
     {
     case oSTART:
        Start(PIDid);  
        break;
     case oOFF:
        Off(PIDid);
        break;
     case oRUN:
        Run(PIDid);
        break;
     case oAUTO:
        Auto(PIDid);
        break;
     case oMAN:
        Man(PIDid);
        break;
     }
   }
}

// MODE LOOPS ----------------------------------------------
void Start(int PIDid) 
{
   UpdateDisplayLabels();
   opState[PIDid] = oRUN; // start control 
}

void Off(int PIDid)
{
   PIDs[PIDid]->SetMode(MANUAL);
  // SaveParameters();
   PIDData[PIDid]->Output = 0;
   digitalWrite(relay[PIDid], LOW);  // make sure it is off
   onTime[PIDid] =  PIDData[PIDid]->Output; 
}

void Run(int PIDid)
{

  //Serial.print(PIDData[PIDid]->deviceAddress[0]);
  if (PIDData[PIDid]->deviceAddress[0] ==0)  // check for no sensor configured 
  {
    opState[PIDid] = oOFF;
  }
  else{
/*  if (Element < PIDData[PIDid]->Setpoint)
  {
   PIDs[PIDid]->SetMode(MANUAL);
//   SaveParameters();
//   PIDs[PIDid]->SetTunings(Kp,Ki,Kd);
   
   PIDData[PIDid]->Output =  PIDData[PIDid]->WindowSize; 
  }
  else*/
  {
   PIDs[PIDid]->SetMode(AUTOMATIC);
//   SaveParameters();
//   PIDs[PIDid]->.SetTunings(Kp,Ki,Kd);
   DoControl(PIDid);
   opState[PIDid] = oAUTO;
  } 
   onTime[PIDid] =  PIDData[PIDid]->Output;  
}
}
void Auto(int PIDid)
{
   PIDs[PIDid]->SetMode(AUTOMATIC);
//   SaveParameters();
//   myPID.SetTunings(Kp,Ki,Kd);
   DoControl(PIDid);  
   onTime[PIDid] =  PIDData[PIDid]->Output; 
}

void Man (int PIDid)
{
  PIDs[PIDid]->SetMode(MANUAL);
//  SaveParameters();
 onTime[PIDid] =  PIDData[PIDid]->Output;  
}



// ************************************************
// Execute the control loop
// ************************************************
void DoControl(int PIDid)
{
#ifdef TUNE 
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
        FinishAutoTune();
     }
  }
  else // Execute control algorithm
  {
  #endif  
     PIDs[PIDid]->Compute();
  #ifdef TUNE   
  }
  #endif
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime[PIDid] =  PIDData[PIDid]->Output; 
}
#ifdef TUNE
// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune(int PIDid)
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

// ************************************************
// Return to normal control
// ************************************************
void FinishAutoTune(int PIDid)
{
   tuning = false;

   // Extract the auto-tune calculated parameters
   Kp = aTune.GetKp();
   Ki = aTune.GetKi();
   Kd = aTune.GetKd();

   // Re-tune the PID and revert to normal control mode
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetMode(ATuneModeRemember);
   
   // Persist any changed parameters to EEPROM
   SaveParameters();
}
#endif
