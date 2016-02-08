 
#include <TFT.h>               // Arduino LCD library
#include <SPI.h>
#include <OneWire.h>           // Comms for DallasTemperature
#include <DallasTemperature.h> // DS18B20 Library - Alarms turned off in .h file.
#include <PID_v1.h>            // PID Library
#include <EEPROM.h>

// ------------------------------------------------------
// ---PINS-----------------------------------------------
// ------------------------------------------------------
#define pwm   6
#define relay 7

#define cs   10
#define dc    9
#define rst   4
#define sd_cs 8

#define ONE_WIRE_BUS 5          // Data wire is plugged into pin 5 on the Arduino
#define TEMPERATURE_PRECISION 12

// ------------------------------------------------------
// ----KEYBOARD/BUTTONS CONSTANTS------------------------
// ------------------------------------------------------
#define BUTTON_BACK 1
#define BUTTON_ENTER 2
#define BUTTON_UP 4
#define BUTTON_DOWN 8

#define KB_DeadBand  5
#define KB_0000 0
#define KB_0001 730
#define KB_0010 682
#define KB_0011 865
#define KB_0100 613
#define KB_0101 865
#define KB_0110 828
#define KB_0111 926
#define KB_1000 513
#define KB_1001 852
#define KB_1010 818
#define KB_1011 925
#define KB_1100 767
#define KB_1101 920
#define KB_1110 896
#define KB_1111 959

unsigned long lastInput = 0; // last button press - used for keyboard timeout

// ------------------------------------------------------
// EEPROM addresses for persisted data
// ------------------------------------------------------
#define SpAddress       0
#define KpSlaveAddress  8
#define KiSlaveAddress  16
#define KdSlaveAddress  24
#define KpMasterAddress 32
#define KiMasterAddress 40
#define KdMasterAddress 48
#define TrendAddress    56
#define TickAddress     64
#define AmbientAddress  65
#define ElementAddress  66
#define FluidAddress    67
#define SampleTimeAddress 68
#define MasterOPMinAddress 69
#define MasterOPMaxAddress 70

// ------------------------------------------------------
// ------PID Variables-----------------------------------
// ------------------------------------------------------
double SetpointSlave, SetpointMaster,lastSetpoint = 0; 
double InputSlave, InputMaster, Ambient, Element, Fluid;
double lastInputMaster, lastAmbient, lastElement;
double lastOutput = 0;
double OutputSlave =0;
double OutputMaster =0;

// pid tuning parameters
double KpSlave, KpMaster;
double KiSlave, KiMaster;
double KdSlave, KdMaster;
// pid run parameters
byte   SampleTime  = 30;
byte   MasterOPMin = 15;
byte   MasterOPMax = 35;

//Specify the links and initial tuning parameters
PID myPIDSLAVE (&InputSlave,  &OutputSlave,  &SetpointSlave,  KpSlave,  KiSlave,  KdSlave,  DIRECT);
PID myPIDMASTER(&InputMaster, &OutputMaster, &SetpointMaster, KpMaster, KiMaster, KdMaster, DIRECT);
// ------------------------------------------------------
// ----INPUT VARIABLES ---------------------------------
// ------------------------------------------------------

byte Fluidselected   = 2;
byte Ambientselected = 1;
byte Elementselected = 0;


// ------------------------------------------------------
// ----OUTPUT VARIABLES ---------------------------------
// ------------------------------------------------------
unsigned long WindowSize = 100000; 
unsigned long windowStartTime;
volatile unsigned long onTime = 0;
boolean      Output_Status = false;
// ------------------------------------------------------

// ------------------------------------------------------
// States for Operating Mode, Menu Location and Display Range
// ------------------------------------------------------
enum operatingState { oSTART = 0, OFF, RUN, CASC, AUTO, MAN};
enum menuState      { mSTART = 0, mMODE_OFF, mMODE_RUN, mMODE_CASC, mMODE_AUTO, mMODE_MAN,  
                      mMODE, mSP, mOP,mTUNE, mPARAMS,
                      mSP_CHANGE,mOP_CHANGE,
                      mPARAMS_PID, mPARAMS_PID_MOPMAX,        mPARAMS_PID_MOPMIN,        mPARAMS_PID_STIME,
                                   mPARAMS_PID_MOPMAX_CHANGE, mPARAMS_PID_MOPMIN_CHANGE, mPARAMS_PID_STIME_CHANGE,
                      mPARAMS_SENSORS, mPARAMS_SENSORS_MASTER_IN,        mPARAMS_SENSORS_SLAVE_IN,        mPARAMS_SENSORS_AMBIENT_IN,
                                       mPARAMS_SENSORS_MASTER_IN_CHANGE, mPARAMS_SENSORS_SLAVE_IN_CHANGE, mPARAMS_SENSORS_AMBIENT_IN_CHANGE,
                      mTUNE_MASTER,  mTUNE_MASTER_P,        mTUNE_MASTER_I,        mTUNE_MASTER_D,
                                     mTUNE_MASTER_P_CHANGE, mTUNE_MASTER_I_CHANGE, mTUNE_MASTER_D_CHANGE,
                      mTUNE_SLAVE,   mTUNE_SLAVE_P,         mTUNE_SLAVE_I,         mTUNE_SLAVE_D,  
                                     mTUNE_SLAVE_P_CHANGE,  mTUNE_SLAVE_I_CHANGE,  mTUNE_SLAVE_D_CHANGE,
                      mPARAMS_DISPLAY,               
                      mPARAMS_DISPLAY_BRIGHTNESS, mPARAMS_DISPLAY_BRIGHTNESS_CHANGE, mPARAMS_DISPLAY_RESTART,               
                      mPARAMS_DISPLAY_SCALE,mDISPLAY_1_15,mDISPLAY_2_15,mDISPLAY_4_15,mDISPLAY_6_10,mDISPLAY_12_25,mDISPLAY_23_13,mDISPLAY_45_20,
                      mDISPLAY_68_13,mDISPLAY_90_10,mDISPLAY_113_16,mDISPLAY_135_13,mDISPLAY_180_10,mDISPLAY_270_13,mDISPLAY_540_13,mDISPLAY_1080_13};
operatingState opState = oSTART;
menuState      mState = mSTART;
menuState      dState = mDISPLAY_1_15;

double  doublevalue;  // used for editing parameters
byte    bytevalue;    // used for editing parameters

// ------------------------------------------------------
// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
// ------------------------------------------------------
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
DeviceAddress Thermometer0, Thermometer1, Thermometer2;
double Temperature0,Temperature1,Temperature2;
int sensorcount;

// ------------------------------------------------------
// --Display Variables ----------------------------------
// ------------------------------------------------------

#define YMIN 18
#define YMAX 118
#define XMIN 0
#define XMAX 159

TFT TFTscreen = TFT(cs, dc, rst);
// Trend Variable
long lastTrendTime = 0;
long trendInterval = 1875L; // trend every 30 seconds
int tickInterval = 16; // trend every 30 seconds
byte brightness = 128;

int   x_pos =0; //position along the graph x axis
boolean lastOutput_Status = false;

// ------------------------------------------------------
// -----INITIALIZATION-----------------------------------
// ------------------------------------------------------
void setup() { 
  // Setting up Pins
    pinMode(relay, OUTPUT);
  digitalWrite(relay, LOW);  // make sure it is off to start
  pinMode(10, OUTPUT);  
  pinMode(pwm, OUTPUT);
  analogWrite(pwm, brightness);
  
  // initialize the display
  TFTscreen.begin();
  TFTscreen.setRotation(3);
  TFTscreen.background(0,0,0); 
  TFTscreen.setTextSize(1);
  TFTscreen.setCursor(0, 0); 
  
  // setting up Dallas Sensors
  sensors.begin();
  sensorcount =  sensors.getDeviceCount();
  sensors.setWaitForConversion(false); 
  if (sensorcount)
  {
    if (sensors.getAddress(Thermometer0, 0)) {sensors.setResolution(Thermometer0, TEMPERATURE_PRECISION); printAddress(Thermometer0); TFTscreen.println(" ");}  
    if (sensors.getAddress(Thermometer1, 1)) {sensors.setResolution(Thermometer1, TEMPERATURE_PRECISION); printAddress(Thermometer1); TFTscreen.println(" ");} 
    if (sensors.getAddress(Thermometer2, 2)) {sensors.setResolution(Thermometer2, TEMPERATURE_PRECISION); printAddress(Thermometer2); TFTscreen.println(" ");}
  } else
  { 
    TFTscreen.println("DS18B20 error");  
  }

//  Setting up PID
   LoadParameters();
   myPIDSLAVE.SetTunings(KpSlave,KiSlave,KdSlave);
   myPIDSLAVE.SetSampleTime(SampleTime*1000L);
   myPIDSLAVE.SetOutputLimits(0.0, 100.0);
   myPIDMASTER.SetTunings(KpMaster,KiMaster,KdMaster);
   myPIDMASTER.SetSampleTime(SampleTime*1000L);
   myPIDMASTER.SetOutputLimits(1.0*MasterOPMin, 1.0*MasterOPMax);

    // Output Interrupt - Setting up  
    // Run timer2 interrupt every 15 ms 
    TCCR2A = 0;
    TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
  
    //Timer2 Overflow Interrupt Enable
    TIMSK2 |= 1<<TOIE2;
    
    
    // All Done
    delay(2000);
}

// ------------------------------------------------------
// -----RUNNING LOOP  -----------------------------------
// ------------------------------------------------------
void loop() {
   
   UpdateSensors();         // get new values
   switch (opState)         // run control depending on op mode
   {
   case oSTART:
      Start();  
      break;
   case OFF:
      Off();
      break;
   case RUN:
      Run();
      break;
   case CASC:
      Casc();
      break;
   case AUTO:
      Auto();
      break;
   case MAN:
      Man();
      break;
   }
   UpdateTrends();
   UpdateDisplayValues();
   Menu();                  // run menu system, display menu and update labels on keypress
  // All done ----------------------------------------------
} // LOOP ALL DONE -----------------------------------------

// ------------------------------------------------------
// -----Update Sensor Values and copy to PIDs -----------
// ------------------------------------------------------
void UpdateSensors()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
    if (sensorcount >2 ) {Temperature2 = sensors.getTempC(Thermometer2);}
    if (sensorcount >1 ) {Temperature1 = sensors.getTempC(Thermometer1);}
    if (sensorcount >0 ) {Temperature0 = sensors.getTempC(Thermometer0);}
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  switch (Fluidselected)
  {
  case 0:
        Fluid = Temperature0;
    break;  
  case 1:
        Fluid = Temperature1;
    break;
  case 2:
        Fluid = Temperature2;
    break;
  }
 
  switch (Elementselected)
  {
  case 0:
        Element = Temperature0;
    break;  
  case 1:
        Element = Temperature1;
    break;
  case 2:
        Element = Temperature2;
    break;
 
  }
  
  switch (Ambientselected)
  {
  case 0:
        Ambient = Temperature0;
    break;  
  case 1:
        Ambient = Temperature1;
    break;
  case 2:
        Ambient = Temperature2;
    break;
  }
  InputMaster=Fluid;
  InputSlave=Element;
}

// ---------------------------------------------------------
// MODE LOOPS ----------------------------------------------
// ---------------------------------------------------------
void Start() 
{
   TFTscreen.background(0,0,0);   //erase display
   UpdateDisplayLabels();         // force a label update on start
   UpdateDisplayDiv(); 
   UpdateDisplayMode();
   opState = RUN;                 // start control in RUN mode
}

void Off()
{
   myPIDSLAVE.SetMode(MANUAL);
   myPIDMASTER.SetMode(MANUAL);
   SaveParameters();
   OutputMaster = Element;     //set output as actual element temperature
   SetpointSlave=OutputMaster; // 
   OutputSlave = 0;            // turn off heat
   digitalWrite(relay, LOW);   // make sure it is off
   onTime = OutputSlave * WindowSize/100; 
}

void Run()
{
  if (Element < SetpointMaster)  // Run with heat at 100% until element reaches Master SP temperature
  {
   myPIDSLAVE.SetMode(MANUAL);
   myPIDMASTER.SetMode(MANUAL);
   SaveParameters();
   myPIDSLAVE.SetTunings(KpSlave,KiSlave,KdSlave);
   myPIDMASTER.SetTunings(KpMaster,KiMaster,KdMaster);
   OutputMaster = Element;      //set output as actual element temperature
   SetpointSlave=OutputMaster;
   OutputSlave = 100;           // make sure it is 100% on.
  }
  else
  {
   opState = CASC;              // switch to Cascade control once element has preheated
  } 
  onTime = OutputSlave * WindowSize/100; 
}
void Casc()  // run PIDMaster -> PIDSlave
{
   myPIDSLAVE.SetMode(AUTOMATIC);
   myPIDMASTER.SetMode(AUTOMATIC);
   SaveParameters();
   myPIDSLAVE.SetTunings(KpSlave,KiSlave,KdSlave);
   myPIDMASTER.SetTunings(KpMaster,KiMaster,KdMaster);
   myPIDMASTER.Compute();
   myPIDSLAVE.Compute();
   // Time Proportional relay state is updated regularly via timer interrupt.
   onTime = OutputSlave * WindowSize/100; 
}

void Auto()  // run SPMaster -> PIDSlave 
{
   myPIDSLAVE.SetMode(AUTOMATIC);
   myPIDMASTER.SetMode(MANUAL);
   SaveParameters();
   myPIDSLAVE.SetTunings(KpSlave,KiSlave,KdSlave);
   myPIDMASTER.SetTunings(KpMaster,KiMaster,KdMaster);
   OutputMaster =SetpointMaster;
   SetpointSlave=OutputMaster;
   myPIDMASTER.Compute();
   myPIDSLAVE.Compute();
   // Time Proportional relay state is updated regularly via timer interrupt.
   onTime = OutputSlave * WindowSize/100; 
}

void Man ()  // run all manual
{
  myPIDSLAVE.SetMode(MANUAL);
  myPIDMASTER.SetMode(MANUAL);
  SaveParameters();
  OutputMaster = Element;      //set output as actual element temperature
  SetpointSlave=OutputMaster;
  onTime = OutputSlave * WindowSize/100; 
}

// ------------------------------------------------------
// ----- MENU SYSTEM ------------------------------------
// ------------------------------------------------------
void Menu()
{
   volatile uint8_t buttons =   ReadButtons();
   DisplayMenu();  // always display menu
   if (buttons) 
   { 
     volatile uint8_t endbuttons = buttons;
     while(endbuttons) {delay(20); endbuttons =  ReadButtons();}
     switch (mState)
     {
     case mSTART: 
       // Start();  
        if (buttons & BUTTON_UP)    {
          switch (opState)
               {
               case oSTART:
                  ;  
                  break;
               case OFF:
                  mState = mMODE;
                  break;
               case RUN:
                  mState = mSP;
                  break;
               case AUTO:
                  mState = mSP;
                  break;
               case MAN:
                  mState = mOP;
                  break;
               }}
        if (buttons & BUTTON_DOWN)  
          {
               switch (opState)
               {
               case oSTART:
                  ;  
                  break;
               case OFF:
                  mState = mMODE;
                  break;
               case RUN:
                  mState = mSP;
                  break;
               case AUTO:
                  mState = mSP;
                  break;
               case MAN:
                  mState = mOP;
                  break;
               }
          }   
        if (buttons & BUTTON_BACK)  {;}
        if (buttons & BUTTON_ENTER) {;}
        break;
     case mMODE:
        //Mode(); 
        if (buttons & BUTTON_UP)   {
              switch (opState)
               {
               case oSTART:
                  ;  
                  break;
               case OFF:
                  mState = mSTART;
                  break;
               case RUN:
                  mState = mSP;
                  break;
               case AUTO:
                  mState = mSP;
                  break;
               case MAN:
                  mState = mOP;
                  break;
               }
          }
        if (buttons & BUTTON_DOWN) 
          {
            mState = mTUNE;
          }   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){
            switch (opState)
               {
               case oSTART:
                  mState = mMODE_OFF;
                  break;
               case OFF:
                  mState = mMODE_OFF;
                  break;
               case RUN:
                  mState = mMODE_RUN;
                  break;
               case AUTO:
                  mState = mMODE_AUTO;
                  break;
               case MAN:
                  mState = mMODE_MAN;
                  break;
               }
            }
        break;
      case mMODE_OFF:
        if (buttons & BUTTON_UP)   {mState = mMODE_MAN;}
        if (buttons & BUTTON_DOWN) {mState = mMODE_RUN;}   
        if (buttons & BUTTON_BACK) {mState = mMODE;}
        if (buttons & BUTTON_ENTER){opState = OFF; mState=mSTART; }      
        break;
      case mMODE_RUN:
        if (buttons & BUTTON_UP)   {mState = mMODE_OFF;}
        if (buttons & BUTTON_DOWN) {mState = mMODE_CASC;}   
        if (buttons & BUTTON_BACK) {mState = mMODE;}
        if (buttons & BUTTON_ENTER){opState = RUN; mState=mSTART; }        
        break;
      case mMODE_CASC:
        if (buttons & BUTTON_UP)   {mState = mMODE_RUN;}
        if (buttons & BUTTON_DOWN) {mState = mMODE_AUTO;}   
        if (buttons & BUTTON_BACK) {mState = mMODE;}
        if (buttons & BUTTON_ENTER){opState = CASC; mState=mSTART; }        
        break;  
      case mMODE_AUTO:
        if (buttons & BUTTON_UP)   {mState = mMODE_RUN;}
        if (buttons & BUTTON_DOWN) {mState = mMODE_MAN;}   
        if (buttons & BUTTON_BACK) {mState = mMODE;}
        if (buttons & BUTTON_ENTER){opState = AUTO; mState=mSTART;}              
        break;
      case mMODE_MAN:
        if (buttons & BUTTON_UP)   {mState = mMODE_AUTO;}
        if (buttons & BUTTON_DOWN) {mState = mMODE_OFF;}   
        if (buttons & BUTTON_BACK) {mState = mMODE;}
        if (buttons & BUTTON_ENTER){opState = MAN; mState=mSTART;}              
        break;
      case mTUNE:
        if (buttons & BUTTON_UP)   {mState = mMODE;}
        if (buttons & BUTTON_DOWN) { mState = mPARAMS;}   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_MASTER;}
        break;  
      case mTUNE_MASTER:
        if (buttons & BUTTON_UP)   {mState = mTUNE_SLAVE;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_SLAVE;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_MASTER_P;}
     case mTUNE_MASTER_P:
        if (buttons & BUTTON_UP)   {mState = mTUNE_MASTER_I;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_MASTER_I;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_MASTER;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_MASTER_P_CHANGE; doublevalue = KpMaster;}
        break;
     case mTUNE_MASTER_P_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_MASTER_P;}
        if (buttons & BUTTON_ENTER){mState = mSTART; KpMaster = doublevalue; SaveParameters();}
        break;       
     case mTUNE_MASTER_I:
        if (buttons & BUTTON_UP)   {mState = mTUNE_MASTER_P;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_MASTER_D;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_MASTER;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_MASTER_I_CHANGE; doublevalue = KiMaster;}
        break;
     case mTUNE_MASTER_I_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_MASTER_I;}
        if (buttons & BUTTON_ENTER){mState = mSTART; KiMaster = doublevalue; SaveParameters();}
        break;       
     case mTUNE_MASTER_D:
        if (buttons & BUTTON_UP)   {mState = mTUNE_MASTER_I;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_MASTER_P;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_MASTER;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_MASTER_D_CHANGE; doublevalue = KdMaster;}
        break;
     case mTUNE_MASTER_D_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_MASTER_D;}
        if (buttons & BUTTON_ENTER){mState = mSTART; KdMaster = doublevalue; SaveParameters();}
        break;        
      case mTUNE_SLAVE:
        if (buttons & BUTTON_UP)   {mState = mTUNE_MASTER;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_MASTER;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_SLAVE_P;}
     case mTUNE_SLAVE_P:
        if (buttons & BUTTON_UP)   {mState = mTUNE_SLAVE_D;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_SLAVE_I;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_SLAVE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_SLAVE_P_CHANGE; doublevalue = KpSlave;}
        break;
     case mTUNE_SLAVE_P_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_SLAVE_P;}
        if (buttons & BUTTON_ENTER){mState = mSTART; KpSlave = doublevalue; SaveParameters();}
        break;       
     case mTUNE_SLAVE_I:
        if (buttons & BUTTON_UP)   {mState = mTUNE_SLAVE_P;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_SLAVE_D;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_SLAVE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_SLAVE_I_CHANGE; doublevalue = KiSlave;}
        break;
     case mTUNE_SLAVE_I_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_SLAVE_I;}
        if (buttons & BUTTON_ENTER){mState = mSTART; KiSlave = doublevalue; SaveParameters();}
        break;       
     case mTUNE_SLAVE_D:
        if (buttons & BUTTON_UP)   {mState = mTUNE_SLAVE_I;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_SLAVE_P;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_SLAVE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_SLAVE_D_CHANGE; doublevalue = KdSlave;}
        break;
     case mTUNE_SLAVE_D_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_SLAVE_D;}
        if (buttons & BUTTON_ENTER){mState = mSTART; KdSlave = doublevalue; SaveParameters(); }
        break;   
        
     case mPARAMS:
        if (buttons & BUTTON_UP)   {mState = mTUNE;}
        if (buttons & BUTTON_DOWN) {
         switch (opState)
               {
               case oSTART:
                  ;  
                  break;
               case OFF:
                  mState = mMODE;
                  break;
               case RUN:
                  mState = mMODE;
                  break;
               case AUTO:
                  mState = mSP;
                  break;
               case MAN:
                  mState = mOP;
                  break;
               }
        }   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_DISPLAY;}
        break;
  
     case mSP:
        if (buttons & BUTTON_UP)   {mState = mSTART;}
        if (buttons & BUTTON_DOWN) {mState = mMODE;}   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){mState = mSP_CHANGE; doublevalue = SetpointMaster;}
        break;
     case mSP_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mSP;}
        if (buttons & BUTTON_ENTER){mState = mSTART; SetpointMaster = doublevalue; SaveParameters();}
        break;
       
     case mOP:
        if (buttons & BUTTON_UP)   {mState = mSTART;}
        if (buttons & BUTTON_DOWN) {mState = mMODE;}   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){mState = mOP_CHANGE; doublevalue = OutputSlave;}   
        break;
     case mOP_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 1; if (doublevalue >100) {doublevalue =100;}}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 1; if (doublevalue <0) {doublevalue =0;} }   
        if (buttons & BUTTON_BACK) {mState = mOP;}
        if (buttons & BUTTON_ENTER){mState = mSTART; OutputSlave = doublevalue;}
        break;
     case mPARAMS_DISPLAY:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_SENSORS;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_PID;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_DISPLAY_SCALE;}
        break;        
     case mPARAMS_DISPLAY_SCALE:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_DISPLAY_BRIGHTNESS;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_DISPLAY_RESTART;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_DISPLAY;}
        if (buttons & BUTTON_ENTER){mState = dState;}
        break;
      case mPARAMS_DISPLAY_RESTART:  
        if (buttons & BUTTON_UP)   {mState = mPARAMS_DISPLAY_SCALE;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_DISPLAY_BRIGHTNESS;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_DISPLAY;}
        if (buttons & BUTTON_ENTER){x_pos = 0; mState = mSTART;}
        break;
     case mPARAMS_DISPLAY_BRIGHTNESS:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_DISPLAY_RESTART;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_DISPLAY_SCALE;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_DISPLAY;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_DISPLAY_BRIGHTNESS_CHANGE; bytevalue= brightness;}
        break;
     case mPARAMS_DISPLAY_BRIGHTNESS_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >255) {bytevalue =255;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <16)  {bytevalue =16;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_DISPLAY_BRIGHTNESS;}
        if (buttons & BUTTON_ENTER){mState = mSTART; brightness = bytevalue; SaveParameters();}
        break;   
     case mPARAMS_PID:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_DISPLAY;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_SENSORS;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_PID_MOPMAX;}
        break;
     case mPARAMS_PID_MOPMAX:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_PID_STIME;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_PID_MOPMIN;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_PID;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_PID_MOPMAX_CHANGE; bytevalue= MasterOPMax;}
        break;   
     case mPARAMS_PID_MOPMIN:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_PID_MOPMAX;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_PID_STIME;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_PID;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_PID_MOPMIN_CHANGE; bytevalue= MasterOPMin;}
        break;
     case mPARAMS_PID_STIME:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_PID_MOPMIN;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_PID_MOPMAX;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_PID;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_PID_STIME_CHANGE; bytevalue= SampleTime;}
        break;                
      case mPARAMS_PID_MOPMAX_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >50) {bytevalue =50;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <10)  {bytevalue =10;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_PID_MOPMAX;}
        if (buttons & BUTTON_ENTER){mState = mSTART; MasterOPMax = bytevalue; SaveParameters();}
        break;
      case mPARAMS_PID_MOPMIN_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >50) {bytevalue =50;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <10)  {bytevalue =10;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_PID_MOPMIN;}
        if (buttons & BUTTON_ENTER){mState = mSTART; MasterOPMin = bytevalue; SaveParameters();}
        break;
      case mPARAMS_PID_STIME_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >240) {bytevalue =240;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <1)  {bytevalue =1;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_PID_STIME;}
        if (buttons & BUTTON_ENTER){mState = mSTART; SampleTime = bytevalue; SaveParameters();}
        break;           
     case mPARAMS_SENSORS:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_PID;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_DISPLAY;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_SENSORS_MASTER_IN;}
        break;             
     case mPARAMS_SENSORS_MASTER_IN:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_SENSORS_AMBIENT_IN;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_SENSORS_SLAVE_IN;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_SENSORS;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_SENSORS_MASTER_IN_CHANGE;bytevalue= Fluidselected;}
        break; 
     case mPARAMS_SENSORS_SLAVE_IN:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_SENSORS_MASTER_IN;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_SENSORS_AMBIENT_IN;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_SENSORS;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_SENSORS_SLAVE_IN_CHANGE;bytevalue= Elementselected;}
        break; 
     case mPARAMS_SENSORS_AMBIENT_IN:
        if (buttons & BUTTON_UP)   {mState = mPARAMS_SENSORS_SLAVE_IN;}
        if (buttons & BUTTON_DOWN) {mState = mPARAMS_SENSORS_MASTER_IN;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_SENSORS;}
        if (buttons & BUTTON_ENTER){mState = mPARAMS_SENSORS_AMBIENT_IN_CHANGE;bytevalue= Ambientselected;}
        break; 
      case mPARAMS_SENSORS_MASTER_IN_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >2) {bytevalue =2;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <0)  {bytevalue =0;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_SENSORS_MASTER_IN;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Fluidselected = bytevalue; SaveParameters();}
        break;   
      case mPARAMS_SENSORS_SLAVE_IN_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >2) {bytevalue =2;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <0)  {bytevalue =0;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_SENSORS_SLAVE_IN;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Elementselected = bytevalue; SaveParameters();}
        break;   
      case mPARAMS_SENSORS_AMBIENT_IN_CHANGE:
        if (buttons & BUTTON_UP)   {bytevalue = bytevalue + 1; if (bytevalue >2) {bytevalue =2;}}
        if (buttons & BUTTON_DOWN) {bytevalue = bytevalue - 1; if (bytevalue <0)  {bytevalue =0;} }   
        if (buttons & BUTTON_BACK) {mState = mPARAMS_SENSORS_AMBIENT_IN;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Ambientselected = bytevalue; SaveParameters();}
        break;                 
     case mDISPLAY_1_15:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_1080_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_2_15;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_1_15;mState = mSTART;trendInterval = 750L;  tickInterval = 20; SaveParameters();}
        break;
     case mDISPLAY_2_15:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_1_15;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_4_15;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_2_15;mState = mSTART;trendInterval = 1875L;  tickInterval = 16; SaveParameters();}
        break;
     case mDISPLAY_4_15:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_2_15;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_6_10;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_4_15;mState = mSTART;trendInterval = 3750L;  tickInterval = 16; SaveParameters();}
        break;
     case mDISPLAY_6_10:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_4_15;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_12_25;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_6_10;mState = mSTART;trendInterval = 5625L;  tickInterval = 11; SaveParameters();}
        break;
     case mDISPLAY_12_25:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_6_10;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_23_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_12_25;mState = mSTART;trendInterval = 11250L;  tickInterval = 27; SaveParameters();}
        break;
     case mDISPLAY_23_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_12_25;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_45_20;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_23_13;mState = mSTART;trendInterval = 22500L;  tickInterval = 13; SaveParameters();}
        break;
     case mDISPLAY_45_20:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_23_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_68_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_45_20;mState = mSTART;trendInterval = 45000L;  tickInterval = 20; SaveParameters();}
        break;
     case mDISPLAY_68_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_45_20;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_90_10;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_68_13;mState = mSTART;trendInterval = 67500L;  tickInterval = 10; SaveParameters();}
        break;
     case mDISPLAY_90_10:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_68_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_113_16;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_90_10;mState = mSTART;trendInterval = 90000L;  tickInterval = 10; SaveParameters();}
        break;
     case mDISPLAY_113_16:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_90_10;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_135_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_113_16;mState = mSTART;trendInterval = 112500L;  tickInterval = 16; SaveParameters();}
        break;
     case mDISPLAY_135_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_113_16;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_180_10;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_135_13;mState = mSTART;trendInterval = 135000L;  tickInterval = 13; SaveParameters();}
        break;
     case mDISPLAY_180_10:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_135_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_270_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_180_10;mState = mSTART;trendInterval = 180000L;  tickInterval = 10; SaveParameters();}
        break;
     case mDISPLAY_270_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_180_10;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_540_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_270_13;mState = mSTART;trendInterval = 270000L;  tickInterval = 13; SaveParameters();}
        break;
     case mDISPLAY_540_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_270_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_1080_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_540_13;mState = mSTART;trendInterval = 540000L;  tickInterval = 13; SaveParameters();}
        break;
     case mDISPLAY_1080_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_540_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_1_15;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_1080_13;mState = mSTART;trendInterval = 1080000L;  tickInterval = 13; SaveParameters();}
        break;
     }

 }
       if ((millis() - lastInput) > 5000)  // return to RUN after 5 seconds idle
      {
         mState = mSTART;
         // if we pressed a button then we timeout... Force a display update - just to make it clean
         UpdateDisplayLabels();
         UpdateDisplayDiv(); 
         UpdateDisplayMode();
         lastSetpoint = 0;
         lastInputMaster = 0;
         lastOutput = 0;
         lastAmbient = 0;
         lastElement =0;
         return;
      }
}

void DisplayMenu()
{
  TFTscreen.setCursor(0, 8); 
switch (mState)
     {
     case mSTART: 
        TFTscreen.print("        ");
        break;
     case mMODE:
        TFTscreen.print("MODE  ");
        break;
    case mMODE_OFF:
             TFTscreen.print("OFF   ");
        break;
      case mMODE_RUN:
             TFTscreen.print("RUN   ");
        break;
      case mMODE_CASC:
               TFTscreen.print("CASC  ");
        break;
        
      case mMODE_AUTO:
               TFTscreen.print("AUTO  ");
        break;
      case mMODE_MAN:
         TFTscreen.print("MAN   ");
        break;    
      case mSP_CHANGE:           
      case mOP_CHANGE:    
      case mTUNE_MASTER_P_CHANGE:
      case mTUNE_MASTER_I_CHANGE:        
      case mTUNE_MASTER_D_CHANGE:        
      case mTUNE_SLAVE_P_CHANGE:
      case mTUNE_SLAVE_I_CHANGE:        
      case mTUNE_SLAVE_D_CHANGE:              
               TFTscreen.print(doublevalue);
        break;
      case mTUNE:
         TFTscreen.print("TUNE  ");
        break;
      case mTUNE_MASTER:
         TFTscreen.print("MASTER");
        break;
      case mTUNE_SLAVE:
         TFTscreen.print("SLAVE ");
        break;       
     case mPARAMS:
         TFTscreen.print("PARAM ");           
        break;
     case mPARAMS_PID:
         TFTscreen.print("PID CFG");           
        break;        
     case mPARAMS_PID_MOPMIN:
         TFTscreen.print("M OPMin");           
        break;
     case mPARAMS_PID_MOPMAX:
         TFTscreen.print("M OPMax");           
        break;
     case mPARAMS_PID_STIME:
         TFTscreen.print("PID Int");           
        break;
      case mPARAMS_PID_MOPMIN_CHANGE:              
      case mPARAMS_PID_MOPMAX_CHANGE:              
      case mPARAMS_PID_STIME_CHANGE:              
      case mPARAMS_SENSORS_MASTER_IN_CHANGE:
      case mPARAMS_SENSORS_SLAVE_IN_CHANGE:
      case mPARAMS_SENSORS_AMBIENT_IN_CHANGE:
               TFTscreen.print(bytevalue);
        break;        
      case mPARAMS_SENSORS:
         TFTscreen.print("Sensors");           
        break;                
      case mPARAMS_SENSORS_MASTER_IN:
         TFTscreen.print("Fluid In");           
        break;        
      case mPARAMS_SENSORS_SLAVE_IN:
         TFTscreen.print("Elem. In");           
        break;        
      case mPARAMS_SENSORS_AMBIENT_IN:
         TFTscreen.print("Amb. In ");           
        break;        
     case mSP:
         TFTscreen.print("SP    ");           
        break;
     case mOP:
         TFTscreen.print("OP    ");                
        break;
     case mTUNE_MASTER_P:
     case mTUNE_SLAVE_P:
         TFTscreen.print("Kp    ");                     
        break;
     case mTUNE_MASTER_I:
     case mTUNE_SLAVE_I:
         TFTscreen.print("Ti    ");                          
        break;
     case mTUNE_MASTER_D:
     case mTUNE_SLAVE_D:
         TFTscreen.print("Td    ");                          
        break;
     case mPARAMS_DISPLAY:   
              TFTscreen.print("Display");                          
        break;
     case
        mPARAMS_DISPLAY_RESTART:
              TFTscreen.print("Restart");                          
        break;         
     case mPARAMS_DISPLAY_BRIGHTNESS:   
              TFTscreen.print("Bright.");                          
        break;
     case mPARAMS_DISPLAY_SCALE:   
              TFTscreen.print("X Axis ");                          
        break;
     case mDISPLAY_1_15:
              TFTscreen.print("2 Mins ");                          
        break;
     case mDISPLAY_2_15:
              TFTscreen.print("5 Mins ");                          
        break;
     case mDISPLAY_4_15:
              TFTscreen.print("10 Mins");                          
        break;
     case mDISPLAY_6_10:
              TFTscreen.print("15 Mins");                          
        break;
     case mDISPLAY_12_25:
              TFTscreen.print("30 Mins");                          
        break;
     case mDISPLAY_23_13:
              TFTscreen.print("1 Hour ");                          
        break;
     case mDISPLAY_45_20:
              TFTscreen.print("2 Hours");                          
        break;
     case mDISPLAY_68_13:
              TFTscreen.print("3 Hours");                          
        break;
     case mDISPLAY_90_10:
              TFTscreen.print("4 Hours");                          
        break;
     case mDISPLAY_113_16:
              TFTscreen.print("5 Hours");                          
        break;
     case
        mDISPLAY_135_13:
              TFTscreen.print("6 Hours");                          
        break;
     case
        mDISPLAY_180_10:
              TFTscreen.print("8 Hours");                          
        break;
     case
        mDISPLAY_270_13:
              TFTscreen.print("12 Hrs ");                          
        break;
     case
        mDISPLAY_540_13:
              TFTscreen.print("24 Hrs ");                          
        break;
     case
        mDISPLAY_1080_13:
              TFTscreen.print("48 Hrs ");                          
        break;           
     }
}



// Display Operations ----------------------------------------------
void UpdateDisplayLabels()
{
   TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);
   TFTscreen.setCursor(0, 0);
   TFTscreen.print("SP");
   TFTscreen.setTextColor(ST7735_GREEN,ST7735_BLACK);
   TFTscreen.setCursor(56, 0);
   TFTscreen.print("PV");   
   TFTscreen.setTextColor(ST7735_BLUE,ST7735_BLACK);
   TFTscreen.setCursor(110, 0);
   TFTscreen.print("OP"); 
     
}
void UpdateDisplayMode()
{
            TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);
            TFTscreen.setCursor(0, 120);
            switch (opState)
               {
               case oSTART:
                     TFTscreen.print("STRT");
                  break;
               case OFF:
                     TFTscreen.print("OFF ");
                  break;
               case RUN:
                     TFTscreen.print("RUN ");               
                  break;
               case AUTO:
                     TFTscreen.print("AUTO");               
                  break;
               case MAN:
                     TFTscreen.print("MAN ");               
                  break;
               }
}
void UpdateDisplayDiv()
{
            TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);
            TFTscreen.setCursor(96, 120);



            if (trendInterval >= 270000L){
              if (trendInterval == 270000L) {TFTscreen.print(" 1");}            
              if (trendInterval == 540000L) {TFTscreen.print(" 2");}
              if (trendInterval == 1080000L) {TFTscreen.print(" 4");}
              TFTscreen.print(" hrs/div");  
            }
            else
            {
              if (trendInterval >= 3750L)
              {
                if (trendInterval == 3750L) {TFTscreen.print(" 1");}            
                if (trendInterval == 5625L) {TFTscreen.print(" 1");}
                if (trendInterval == 11250L) {TFTscreen.print(" 5");}
                if (trendInterval == 22500L) {TFTscreen.print(" 5");}
                if (trendInterval == 45000L) {TFTscreen.print("15");}                
                if (trendInterval == 67500L) {TFTscreen.print("15");}                 
                if (trendInterval == 90000L) {TFTscreen.print("15");}                 
                if (trendInterval == 112500L) {TFTscreen.print("30");}                
                if (trendInterval == 135000L) {TFTscreen.print("30");}                
                if (trendInterval == 180000L) {TFTscreen.print("30");}                                
                TFTscreen.print(" min/div");                
              }
              else
              {
                if (trendInterval == 750L) {TFTscreen.print("15");}            
                if (trendInterval == 1875L) {TFTscreen.print("30");}            
                TFTscreen.print(" sec/div");                              
              }
            }
}

void UpdateDisplayValues()
{
  if (!(lastSetpoint == SetpointMaster) ){
     lastSetpoint = SetpointMaster;
     TFTscreen.setCursor(14, 0);
     TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);    
     TFTscreen.print(SetpointMaster);
   }

  if (!(lastInputMaster == InputMaster)){    
    lastInputMaster = InputMaster;
    TFTscreen.setTextColor(ST7735_GREEN,ST7735_BLACK);
    TFTscreen.setCursor(70, 0);
    TFTscreen.print(InputMaster);
  } 

   if (!(lastOutput_Status == Output_Status) || !(lastOutput == OutputSlave))
   {
    lastOutput_Status = Output_Status;
    lastOutput = OutputSlave;
    TFTscreen.setCursor(124, 0);
    if (Output_Status)
      {
        TFTscreen.setTextColor(ST7735_BLACK,ST7735_BLUE);
      }
      else
      {
        TFTscreen.setTextColor(ST7735_BLUE,ST7735_BLACK);
      }
     TFTscreen.print(OutputSlave);
   }             
   
   TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);

   if (!(lastAmbient == Ambient))
   {
    lastAmbient = Ambient;
    TFTscreen.setCursor(70, 8);
    TFTscreen.print(Ambient);
   }
   
   if (!(lastElement == Element))
   {
    lastElement = Element;
    TFTscreen.setCursor(124, 8);
    TFTscreen.print(Element);
   }
}

void UpdateTrends()
{
    if ((millis() - lastTrendTime) > trendInterval)  
    {
      int y_pos = 0;
      lastTrendTime = millis();
      x_pos = x_pos + 1;
      if (x_pos > XMAX) {x_pos = XMIN;}
       TFTscreen.fillRect(x_pos, YMIN, 5, YMAX-YMIN+1, ST7735_BLACK);
      if (((x_pos-1)%tickInterval)==0) {
          for (int i=MasterOPMin; i <= MasterOPMax; i++){
            y_pos = map(i*10, MasterOPMin*10, MasterOPMax*10, YMAX, YMIN);
            TFTscreen.drawPixel(x_pos,y_pos , ST7735_YELLOW); //plot single tick mark
          } 
      }
      TFTscreen.drawPixel(x_pos, YMIN-1 , ST7735_WHITE); 
      TFTscreen.drawPixel(x_pos, YMAX+1,  ST7735_WHITE);

      y_pos  = map(SetpointSlave*10, MasterOPMin*10, MasterOPMax*10, YMAX, YMIN);
      TFTscreen.drawPixel(x_pos, y_pos, ST7735_WHITE);   

      y_pos  = map(InputSlave*10, MasterOPMin*10, MasterOPMax*10, YMAX, YMIN);
      TFTscreen.drawPixel(x_pos, y_pos, ST7735_GREEN);

      y_pos = map(Element*10,    MasterOPMin*10, MasterOPMax*10, YMAX, YMIN); 
      TFTscreen.drawPixel(x_pos, y_pos, 0x3DEF);
      
      y_pos = map(Ambient*10,    MasterOPMin*10, MasterOPMax*10, YMAX, YMIN);
      TFTscreen.drawPixel(x_pos, y_pos, 0x318c);
      
      y_pos  = map(OutputSlave*10, 0, 1000, YMAX, YMIN);
      if (Output_Status)
      {
        TFTscreen.drawPixel(x_pos, y_pos, ST7735_RED);
      }
      else
      {
        TFTscreen.drawPixel(x_pos, y_pos, ST7735_BLUE);
      }  
    }
}

// ************************************************
// Check buttons and time-stamp the last press
// ************************************************
uint8_t ReadButtons()
{
  long time =   time = millis();
  int debounce_count = 10; // number of millis/samples to consider before declaring a debounced input
  int counter = 0;       
  uint8_t reading;           // the current value read from the input pin
  uint8_t current_state =0;

  boolean stable = false;
  uint8_t buttons  = 0;
  reading = ReadAnalogButtons();

while (!stable)
{
 if(millis() != time)
  {
    reading = ReadAnalogButtons();

    if(reading == current_state)
    {
      counter++;
    }
    if(reading != current_state)
    {
       counter = 0; 
    }
    // If the Input has shown the same value for long enough let's switch it
    if(counter >= debounce_count)
    {
      buttons =  reading;
      stable = true;
    }
    current_state = reading;
    time = millis();
  }
}
  if (buttons != 0)
  {
    lastInput = millis();
  }
  return buttons;
}

uint8_t ReadAnalogButtons()
{
  
  uint8_t buttons  = 0;
  int sensor = analogRead(A0);
  if (sensor < KB_DeadBand) 
  {
    buttons  = 0;
  } 
  else 
  {
    if  ((sensor < (KB_1000 + KB_DeadBand) )&(sensor > (KB_1000 - KB_DeadBand)))
    {
      buttons  = BUTTON_BACK;
    }
    else 
    {
      if ((sensor < (KB_0100 + KB_DeadBand) )&(sensor > (KB_0100 - KB_DeadBand)))
      {
        buttons  = BUTTON_ENTER;
      }
      else
      {
          if ((sensor < (KB_0010 + KB_DeadBand) )&(sensor > (KB_0010 - KB_DeadBand)))
          {
            buttons  = BUTTON_DOWN;
          }
          else
          {
            if ((sensor < (KB_0001 + KB_DeadBand ))&(sensor > (KB_0001 - KB_DeadBand)))
            {
               buttons  = BUTTON_UP;
            }
          }
      }
    }
  }
  return buttons;
}

// ************************************************
// Timer Interrupt Handler
// ************************************************
SIGNAL(TIMER2_OVF_vect) 
{
  if (opState == OFF)
  {
    digitalWrite(relay, LOW);  // make sure relay is off
  }
  else
  {
    DriveOutput();
  }
}

// ************************************************
// Called by ISR every 15ms to drive the output
// ************************************************
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if((now - windowStartTime)>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if((onTime > 1000) && (onTime > (now - windowStartTime)))
  {
     digitalWrite(relay,HIGH);
     Output_Status = true;
  }
  else
  {
     digitalWrite(relay,LOW);
     Output_Status = false;
  }
}

// ************************************************
// Save any parameter changes to EEPROM
// ************************************************
void SaveParameters()
{
   if (SetpointMaster != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, SetpointMaster);
   }
   if (KpSlave != EEPROM_readDouble(KpSlaveAddress))
   {
      EEPROM_writeDouble(KpSlaveAddress, KpSlave);
   }
   if (KiSlave != EEPROM_readDouble(KiSlaveAddress))
   {
      EEPROM_writeDouble(KiSlaveAddress, KiSlave);
   }
   if (KdSlave != EEPROM_readDouble(KdSlaveAddress))
   {
      EEPROM_writeDouble(KdSlaveAddress, KdSlave);
   }
   if (KpMaster != EEPROM_readDouble(KpMasterAddress))
   {
      EEPROM_writeDouble(KpMasterAddress, KpMaster);
   }
   if (KiMaster != EEPROM_readDouble(KiMasterAddress))
   {
      EEPROM_writeDouble(KiMasterAddress, KiMaster);
   }
   if (KdMaster != EEPROM_readDouble(KdMasterAddress))
   {
      EEPROM_writeDouble(KdMasterAddress, KdMaster);
   }
   
   if (trendInterval != EEPROM_readLong(TrendAddress))
   {
      EEPROM_writeLong(TrendAddress, trendInterval);
   }
   if (tickInterval != EEPROM.read(TickAddress))
   {
      EEPROM.write(TickAddress, tickInterval);
   }

   if (Ambientselected != EEPROM.read(AmbientAddress))
   {
      EEPROM.write(AmbientAddress, Ambientselected);
   }
   if (Elementselected != EEPROM.read(TickAddress))
   {
      EEPROM.write(ElementAddress, Elementselected);
   }
   if (Fluidselected != EEPROM.read(FluidAddress))
   {
      EEPROM.write(FluidAddress, Fluidselected );
   }

}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   SetpointMaster = EEPROM_readDouble(SpAddress);
   KpSlave = EEPROM_readDouble(KpSlaveAddress);
   KiSlave = EEPROM_readDouble(KiSlaveAddress);
   KdSlave = EEPROM_readDouble(KdSlaveAddress);
   KpMaster = EEPROM_readDouble(KpMasterAddress);
   KiMaster = EEPROM_readDouble(KiMasterAddress);
   KdMaster = EEPROM_readDouble(KdMasterAddress);
   
   trendInterval   = EEPROM_readLong(TrendAddress);
   tickInterval    = EEPROM.read(TickAddress);
   Ambientselected = EEPROM.read(AmbientAddress);
   Elementselected = EEPROM.read(ElementAddress);
   Fluidselected   = EEPROM.read(FluidAddress);
   SampleTime      = EEPROM.read(SampleTimeAddress);
   MasterOPMin     = EEPROM.read(MasterOPMinAddress);
   MasterOPMax     = EEPROM.read(MasterOPMaxAddress);
   // Use defaults if EEPROM values are invalid
   if (isnan(SetpointMaster))
   {
     SetpointMaster = 25;
   }
   if (isnan(KpSlave))
   {
     KpSlave = 850;
   }
   if (isnan(KiSlave))
   {
     KiSlave = 10;
   }
   if (isnan(KdSlave))
   {
     KdSlave = 0;
   }  
      if (isnan(KpMaster))
   {
     KpMaster = 1;
   }
   if (isnan(KiMaster))
   {
     KiMaster = 5;
   }
   if (isnan(KdMaster))
   {
     KdMaster = 0;
   } 
   if ((trendInterval==0) || (tickInterval==0))
   {
     trendInterval = 1875L;
     tickInterval = 16;
   }  
   if ((Fluidselected==0) && (Ambientselected==0) && (Elementselected==0))
   {
      Fluidselected   = 2;
      Ambientselected = 1;
      Elementselected = 0;
   }
   if (SampleTime==0) 
    {
      SampleTime = 30;
    }
    if (MasterOPMin==0) 
    {
      MasterOPMin = 15;
    }
    if (MasterOPMax==0) 
    {
      MasterOPMax = 35;
    }
    
}


// ************************************************
// Write floating point values to EEPROM
// ************************************************
void EEPROM_writeDouble(int address, double value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

void EEPROM_writeLong(int address, long value)
{
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      EEPROM.write(address++, *p++);
   }
}

// ************************************************
// Read floating point values from EEPROM
// ************************************************
double EEPROM_readDouble(int address)
{
   double value = 0.0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}

long EEPROM_readLong(int address)
{
   long value = 0;
   byte* p = (byte*)(void*)&value;
   for (int i = 0; i < sizeof(value); i++)
   {
      *p++ = EEPROM.read(address++);
   }
   return value;
}


void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) TFTscreen.print("0");
    TFTscreen.print(deviceAddress[i], HEX);
  }
}
