 
#include "configuration.h"
#include <TFT.h>               // Arduino LCD library
#include <SPI.h>
#ifdef SD
  #include <SD.h>
#endif
#include <OneWire.h>           // Comms for DallasTemperature
#include <DallasTemperature.h> // DS18B20 Library
#include "PID_v1.h"            // PID Library
#ifdef TUNE
  #include <PID_AutoTune_v0.h>   // PID Autotune Library 
#endif

#include <EEPROM.h>


// ------PID Variables------------------------------------------------

double Setpoint, doublevalue;
double Input, Ambient, Element;
double Output =0;

// pid tuning parameters
double Kp;
double Ki;
double Kd;

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
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
boolean tuning = false;
// ************************************************
// States for state machine
// ************************************************
enum operatingState { oSTART = 0, OFF, RUN, AUTO, MAN};
enum menuState      { mSTART = 0, mSP, mOP, mMODE, mMODE_OFF, mMODE_RUN, mMODE_AUTO, mMODE_MAN, mSP_CHANGE,mOP_CHANGE, mTUNE,mPARAMS, mTUNE_P, mTUNE_I, mTUNE_D, mTUNE_P_CHANGE, mTUNE_I_CHANGE, mTUNE_D_CHANGE,mDISPLAY,mRESTART,mDISPLAY_1_15,mDISPLAY_2_15,mDISPLAY_4_15,mDISPLAY_6_10,mDISPLAY_12_25,mDISPLAY_23_13,mDISPLAY_45_20,mDISPLAY_68_13,mDISPLAY_90_10,mDISPLAY_113_16,mDISPLAY_135_13,mDISPLAY_180_10,mDISPLAY_270_13,mDISPLAY_540_13,mDISPLAY_1080_13};
operatingState opState = oSTART;
//operatingState nextopState = OFF;
menuState      mState = mSTART;
menuState      dState = mDISPLAY_1_15;
// 10 second Time Proportional Output window
unsigned long WindowSize = 100000; 
unsigned long windowStartTime;
volatile unsigned long onTime = 50000;
boolean      Output_Status = false;
// ------------------------------------------------------
TFT TFTscreen = TFT(cs, dc, rst);
#ifdef SD
boolean sd_up = false;
#endif
boolean TC_up = false;
unsigned long lastInput = 0; // last button press

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature. 
DallasTemperature sensors(&oneWire);

// arrays to hold device addresses
int sensorcount =0;
DeviceAddress Thermometer0, Thermometer1, Thermometer2;//, Thermometer3;
double Temperature0,Temperature1,Temperature2,Temperature3;
//
const int inputselected = 2;
const int Ambientselected = 1;
const int Elementselected = 0;

// Trend Variable
long lastTrendTime = 0;
long trendInterval = 1875L; // trend every 30 seconds
int tickInterval = 16; // trend every 30 seconds

int x_pos =0; //position along the graph x axis
float y_pos_x; //current graph y axis position of X value
float y_pos_y1; //current graph y axis position of X value
float y_pos_y2; //current graph y axis position of X value
float y_pos_y; //current graph y axis position of Y value
float y_pos_z; //current graph y axis position of Z value

// Display variables
double lastSetpoint = 0;
double lastInputValue = 0;
double lastOutput = 0;
boolean lastOutput_Status = false;
boolean lasttuning = false;

// ------------------------------------------------------
void setup() { 
//char sensorPrintout[10];
  // initialize the serial port
    Serial.begin(9600);
  // Setting up Pins
  pinMode(pwm, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(10, OUTPUT);  
  analogWrite(pwm, 128);
  digitalWrite(relay, LOW);  // make sure it is off to start
  
  // initialize the display
  TFTscreen.begin();
  TFTscreen.setRotation(3);
  TFTscreen.background(0,0,0); 
  TFTscreen.setTextSize(1);
  TFTscreen.setCursor(0, 00); 
//  TFTscreen.println("SETUP");
  
  // setting up Dallas Sensors
  sensors.begin();
  sensorcount =  sensors.getDeviceCount();
  sensors.setWaitForConversion(false);
 // String sensorVal = String(sensorcount);
//  sensorVal.toCharArray(sensorPrintout, 4);

//   TFTscreen.setCursor(0, 00); 
 //  TFTscreen.print("Dallas: ");
//   TFTscreen.println(sensorcount);
 
  if (sensorcount)
  {
    if (sensors.getAddress(Thermometer0, 0)) {sensors.setResolution(Thermometer0, TEMPERATURE_PRECISION); printAddress(Thermometer0); TFTscreen.println(" ");}  
    if (sensors.getAddress(Thermometer1, 1)) {sensors.setResolution(Thermometer1, TEMPERATURE_PRECISION); printAddress(Thermometer1); TFTscreen.println(" ");} 
    if (sensors.getAddress(Thermometer2, 2)) {sensors.setResolution(Thermometer2, TEMPERATURE_PRECISION); printAddress(Thermometer2); TFTscreen.println(" ");}
  //  if (sensors.getAddress(Thermometer3, 3)) {sensors.setResolution(Thermometer3, TEMPERATURE_PRECISION); printAddress(Thermometer3); TFTscreen.println(" ");}
    TC_up = true;
  } else
  { 
    TFTscreen.println("DS18B20 not present");  
    TC_up = false; 
  }

//  Setting up PID
   LoadParameters();
 //  TFTscreen.println("Setting up PID");  
   myPID.SetTunings(Kp,Ki,Kd);
   myPID.SetSampleTime(1000);
   myPID.SetOutputLimits(0, WindowSize);

//  Setting up SD Card
 #ifdef SD 
    
    if (!SD.begin(sd_cs)) 
        {   
          TFTscreen.println("No SD Card");  
          sd_up = false;    
        }
    else 
        {
          sd_up = true;
          TFTscreen.println("SD Card Present"); 
        }
#endif

   // TFTscreen.println("Setting up Interrupt");   
    // Output Interrupt - Setting up  
    // Run timer2 interrupt every 15 ms 
    TCCR2A = 0;
    TCCR2B = 1<<CS22 | 1<<CS21 | 1<<CS20;
  
    //Timer2 Overflow Interrupt Enable
    TIMSK2 |= 1<<TOIE2;
    
    
    // All Done
 //   TFTscreen.println("SETUP - COMPLETE");    
    delay(5000);
}

// ------------------------------------------------------
void loop() {
   //DisplayMenu();
   Menu();
   UpdateSensors();
   UpdateTrends();
   UpdateDisplayLabels();
   UpdateDisplayDiv(); 
   UpdateDisplayValues();
   UpdateDisplayMode();
   switch (opState)
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
   case AUTO:
      Auto();
      break;
   case MAN:
      Man();
      break;
   }
  

#ifdef SD 
  // SD Operations ----------------------------------------------  
  if (sd_up)
  {
    dataString += String(sensor);
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      //Serial.println(dataString);
    }  
    // if the file isn't open, pop up an error:
    else {
      TFTscreen.text("error opening datalog.txt",0,0);
    }
  }
#endif 
  // All done ----------------------------------------------
} // LOOP ALL DONE -----------------------------------------

void UpdateSensors()
{
  // Read the input:
  if (sensors.isConversionAvailable(0))
  {
  //  if (sensorcount >3 ) {Temperature3 = sensors.getTempC(Thermometer3);}
    if (sensorcount >2 ) {Temperature2 = sensors.getTempC(Thermometer2);}
    if (sensorcount >1 ) {Temperature1 = sensors.getTempC(Thermometer1);}
    if (sensorcount >0 ) {Temperature0 = sensors.getTempC(Thermometer0);}
    sensors.requestTemperatures(); // prime the pump for the next one - but don't wait
  }
  switch (inputselected)
  {
  case 0:
      Input = Temperature0;
    break;  
  case 1:
        Input = Temperature1;
    break;
  case 2:
        Input = Temperature2;
    break;
//  case 3:
//        Input = Temperature3;
//    break;  
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
}



void Menu()
{
   volatile uint8_t buttons =   ReadButtons();
   DisplayMenu();
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
        if (buttons & BUTTON_ENTER){opState = OFF; mState=mSTART;}      
        break;
      case mMODE_RUN:
        if (buttons & BUTTON_UP)   {mState = mMODE_OFF;}
        if (buttons & BUTTON_DOWN) {mState = mMODE_AUTO;}   
        if (buttons & BUTTON_BACK) {mState = mMODE;}
        if (buttons & BUTTON_ENTER){opState = RUN; mState=mSTART;}        
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
        if (buttons & BUTTON_ENTER){mState = mTUNE_P;}
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
        if (buttons & BUTTON_ENTER){mState = mDISPLAY;}
        break;
  
     case mSP:
        if (buttons & BUTTON_UP)   {mState = mSTART;}
        if (buttons & BUTTON_DOWN) {mState = mMODE;}   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){mState = mSP_CHANGE; doublevalue = Setpoint;}
        break;
     case mSP_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mSP;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Setpoint = doublevalue;}
        break;
       
     case mOP:
        if (buttons & BUTTON_UP)   {mState = mSTART;}
        if (buttons & BUTTON_DOWN) {mState = mMODE;}   
        if (buttons & BUTTON_BACK) {;}
        if (buttons & BUTTON_ENTER){mState = mOP_CHANGE; doublevalue = map(Output, 0, WindowSize, 0, 1000)/10;}   
        break;
     case mOP_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 1; if (doublevalue >100) {doublevalue =100;}}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 1; if (doublevalue <0) {doublevalue =0;} }   
        if (buttons & BUTTON_BACK) {mState = mOP;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Output = doublevalue * WindowSize/100;}
        break;
     case mTUNE_P:
        if (buttons & BUTTON_UP)   {mState = mTUNE_I;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_I;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_P_CHANGE; doublevalue = Kp;}
        break;
     case mTUNE_P_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_P;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Kp = doublevalue;}
        break;       
     case mTUNE_I:
        if (buttons & BUTTON_UP)   {mState = mTUNE_P;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_D;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_I_CHANGE; doublevalue = Ki;}
        break;
     case mTUNE_I_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_I;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Ki = doublevalue;}
        break;       
     case mTUNE_D:
        if (buttons & BUTTON_UP)   {mState = mTUNE_I;}
        if (buttons & BUTTON_DOWN) {mState = mTUNE_P;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE;}
        if (buttons & BUTTON_ENTER){mState = mTUNE_D_CHANGE; doublevalue = Kd;}
        break;
     case mTUNE_D_CHANGE:
        if (buttons & BUTTON_UP)   {doublevalue = doublevalue + 0.1;}
        if (buttons & BUTTON_DOWN) {doublevalue = doublevalue - 0.1;}   
        if (buttons & BUTTON_BACK) {mState = mTUNE_D;}
        if (buttons & BUTTON_ENTER){mState = mSTART; Kd = doublevalue;}
        break;
     case mDISPLAY:
        if (buttons & BUTTON_UP)   {mState = mRESTART;}
        if (buttons & BUTTON_DOWN) {mState = mRESTART;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){mState = dState;}
        break;
      case mRESTART:  
        if (buttons & BUTTON_UP)   {mState = mDISPLAY;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){x_pos = 0; mState = mSTART;}

        break;
        
     case mDISPLAY_1_15:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_1080_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_2_15;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_1_15;mState = mSTART;trendInterval = 750L;  tickInterval = 20;}
        break;
     case mDISPLAY_2_15:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_1_15;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_4_15;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_2_15;mState = mSTART;trendInterval = 1875L;  tickInterval = 16;}
        break;
     case mDISPLAY_4_15:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_2_15;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_6_10;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_4_15;mState = mSTART;trendInterval = 3750L;  tickInterval = 16;}
        break;
     case mDISPLAY_6_10:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_4_15;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_12_25;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_6_10;mState = mSTART;trendInterval = 5625L;  tickInterval = 11;}
        break;
     case mDISPLAY_12_25:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_6_10;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_23_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_12_25;mState = mSTART;trendInterval = 11250L;  tickInterval = 27;}
        break;
     case mDISPLAY_23_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_12_25;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_45_20;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_23_13;mState = mSTART;trendInterval = 22500L;  tickInterval = 13;}
        break;
     case mDISPLAY_45_20:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_23_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_68_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_45_20;mState = mSTART;trendInterval = 45000L;  tickInterval = 20;}
        break;
     case mDISPLAY_68_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_45_20;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_90_10;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_68_13;mState = mSTART;trendInterval = 67500L;  tickInterval = 10;}
        break;
     case mDISPLAY_90_10:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_68_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_113_16;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_90_10;mState = mSTART;trendInterval = 90000L;  tickInterval = 10;}
        break;
     case mDISPLAY_113_16:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_90_10;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_135_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_113_16;mState = mSTART;trendInterval = 112500L;  tickInterval = 16;}
        break;
     case mDISPLAY_135_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_113_16;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_180_10;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_135_13;mState = mSTART;trendInterval = 135000L;  tickInterval = 13;}
        break;
     case mDISPLAY_180_10:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_135_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_270_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_180_10;mState = mSTART;trendInterval = 180000L;  tickInterval = 10;}
        break;
     case mDISPLAY_270_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_180_10;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_540_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_270_13;mState = mSTART;trendInterval = 270000L;  tickInterval = 13;}
        break;
     case mDISPLAY_540_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_270_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_1080_13;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_540_13;mState = mSTART;trendInterval = 540000L;  tickInterval = 13;}
        break;
     case mDISPLAY_1080_13:
        if (buttons & BUTTON_UP)   {mState = mDISPLAY_540_13;}
        if (buttons & BUTTON_DOWN) {mState = mDISPLAY_1_15;}   
        if (buttons & BUTTON_BACK) {mState = mPARAMS;}
        if (buttons & BUTTON_ENTER){dState = mDISPLAY_1080_13;mState = mSTART;trendInterval = 1080000L;  tickInterval = 13;}
        break;
     }
     //DisplayMenu();
 }
       if ((millis() - lastInput) > 5000)  // return to RUN after 3 seconds idle
      {
         mState = mSTART;
        // TFTscreen.setCursor(0, 8); 
       //  TFTscreen.print("    ");
         return;
      }
}

void DisplayMenu()
{
  TFTscreen.setCursor(0, 8); 
switch (mState)
     {
     case mSTART: 
        TFTscreen.print("       ");
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
      case mMODE_AUTO:
               TFTscreen.print("AUTO  ");
        break;
      case mMODE_MAN:
         TFTscreen.print("MAN   ");
        break;    
      case mSP_CHANGE:    
         TFTscreen.print(doublevalue);
        break;
      case mOP_CHANGE:    
         TFTscreen.print(doublevalue);
        break;
      case mTUNE_P_CHANGE:
               TFTscreen.print(doublevalue);
        break;
      case mTUNE_I_CHANGE:        
               TFTscreen.print(doublevalue);
        break;
      case mTUNE_D_CHANGE:        
               TFTscreen.print(doublevalue);
        break;
      case mTUNE:
         TFTscreen.print("TUNE  ");
        break;
     case mPARAMS:
         TFTscreen.print("PARAM ");           
        break;
     case mSP:
         TFTscreen.print("SP    ");           
        break;
     case mOP:
         TFTscreen.print("OP    ");                
        break;
     case mTUNE_P:
         TFTscreen.print("Kp    ");                     
        break;
     case mTUNE_I:
         TFTscreen.print("Ki    ");                          
        break;
     case mTUNE_D:
         TFTscreen.print("Kd    ");                          
        break;
     case mDISPLAY:   
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
     case
        mRESTART:
              TFTscreen.print("Restart");                          
        break;        
     }
}

// MODE LOOPS ----------------------------------------------
void Start() 
{
  // analogWrite(pwm, 255);
  // digitalWrite(led, HIGH);   // turn the LED on (HIGH is the voltage level)
  // digitalWrite(relay, HIGH);   // turn the LED on (HIGH is the voltage level)
  // delay(100);               // wait for a second
  // analogWrite(pwm, 16);
  // digitalWrite(led, LOW);    // turn the LED off by making the voltage LOW
  // digitalWrite(relay, LOW);    // turn the LED off by making the voltage LOW
  // delay(100);               // wait for a second
   TFTscreen.background(0,0,0);
   UpdateDisplayLabels();
   opState = RUN; // start control 
}

void Off()
{
   myPID.SetMode(MANUAL);
   SaveParameters();
   Output = 0;
   digitalWrite(relay, LOW);  // make sure it is off
   onTime = Output; 
}

void Run()
{
  if (Element < Setpoint)
  {
   myPID.SetMode(MANUAL);
   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   Output = WindowSize;
  }
  else
  {
   myPID.SetMode(AUTOMATIC);
   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   DoControl();
   opState = AUTO;
  } 
   onTime = Output; 
}
void Auto()
{
   myPID.SetMode(AUTOMATIC);
   SaveParameters();
   myPID.SetTunings(Kp,Ki,Kd);
   DoControl();  
   onTime = Output; 
}

void Man ()
{
  myPID.SetMode(MANUAL);
  SaveParameters();
  onTime = Output; 
}



// ************************************************
// Execute the control loop
// ************************************************
void DoControl()
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
     myPID.Compute();
  #ifdef TUNE   
  }
  #endif
  // Time Proportional relay state is updated regularly via timer interrupt.
  onTime = Output; 
}
#ifdef TUNE
// ************************************************
// Start the Auto-Tuning cycle
// ************************************************

void StartAutoTune()
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
void FinishAutoTune()
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
  float pct = map(Output, 0, WindowSize, 0, 10000);
 // if (!(lastSetpoint == Setpoint) || !(lasttuning == tuning)){
 //   lastSetpoint = Setpoint;
 //   lasttuning = tuning;
    // TFTscreen.setCursor(12, 0);
    // if (tuning)
   //   {
  //      TFTscreen.setTextColor(ST7735_BLACK,ST7735_YELLOW);
     // }
    //  else
   //   {
  //      TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);
 //     }    
//     TFTscreen.print(Setpoint);
  //}


  if (!(lastInputValue == Input)){    
    
         TFTscreen.setCursor(14, 0);
        TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);    
             TFTscreen.print(Setpoint);
    lastInputValue = Input;
    TFTscreen.setTextColor(ST7735_GREEN,ST7735_BLACK);
    TFTscreen.setCursor(70, 0);
    TFTscreen.print(Input);
  } 

  if (!(lastOutput_Status == Output_Status) || !(lastOutput == Output)){
  lastOutput_Status = Output_Status;
  lastOutput = Output;
   TFTscreen.setCursor(124, 0);
    if (Output_Status)
      {
        TFTscreen.setTextColor(ST7735_BLACK,ST7735_BLUE);
      }
      else
      {
        TFTscreen.setTextColor(ST7735_BLUE,ST7735_BLACK);
      }

     TFTscreen.print(pct/100);

  }             
    TFTscreen.setTextColor(ST7735_WHITE,ST7735_BLACK);
    TFTscreen.setCursor(70, 8);
    TFTscreen.print(Ambient);
    TFTscreen.setCursor(124, 8);
    TFTscreen.print(Element);

}

void UpdateTrends()
{
      if ((millis() - lastTrendTime) > trendInterval)  
      {
        float pct = map(Output, 0, WindowSize, 0, 1000);
//        y_pos_x = (-(Setpoint-10)*6 + 128); //values to use when displaying on LCD, these are floats, for more precision!

//        y_pos_y = (-(Input-10)*6 + 128); // 120 is axis, so value is displacement from axis, scaled for better visisility!
//        y_pos_y1 = (-(Element-10)*6 + 128); // 120 is axis, so value is displacement from axis, scaled for better visisility!
//        y_pos_y2 = (-(Ambient-10)*6 + 128); // 120 is axis, so value is displacement from axis, scaled for better visisility!
  //      y_pos_z = (-pct/10 + 128);
        y_pos_y  = map(Input*10, 100, 300, 118, 18);
        y_pos_y1 = map(Element*10, 100, 300, 118, 18);
        y_pos_y2 = map(Ambient*10, 100, 300, 118, 18);
        y_pos_x  = map(Setpoint*10, 100, 300, 118, 18);
        y_pos_z  = map(Output, 0, WindowSize, 118, 18);

        
      lastTrendTime = millis();
      x_pos = x_pos + 1;
      if (x_pos > 159) {x_pos = 0;}
       TFTscreen.fillRect(x_pos, 18, 5, 100, ST7735_BLACK);
      if (((x_pos-1)%tickInterval)==0) {
          for (int i=10; i <= 30; i++){
            //TFTscreen.drawPixel(x_pos, (-(i-10)*6 + 128), ST7735_YELLOW); //plot single point 
            int pos = map(i, 10, 30, 118, 18);
            TFTscreen.drawPixel(x_pos,pos , ST7735_YELLOW); //plot single point 
          } 
      }

  //    if (tuning)
  //    {
   //     TFTscreen.drawPixel(x_pos, y_pos_x, ST7735_YELLOW); //plot single point 
   //   }
   //   else
   //   {
        TFTscreen.drawPixel(x_pos, y_pos_x, ST7735_WHITE); //plot single point 
    //  }   
      TFTscreen.drawPixel(x_pos, 17, ST7735_WHITE); 
      TFTscreen.drawPixel(x_pos, 118, ST7735_WHITE);
      TFTscreen.drawPixel(x_pos, y_pos_y, ST7735_GREEN);
      TFTscreen.drawPixel(x_pos, y_pos_y1, 0x3DEF);
      TFTscreen.drawPixel(x_pos, y_pos_y2, 0x318c);
  
      if (Output_Status)
      {
        TFTscreen.drawPixel(x_pos, y_pos_z, ST7735_RED);
      }
      else
      {
        TFTscreen.drawPixel(x_pos, y_pos_z, ST7735_BLUE);
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
   if (Setpoint != EEPROM_readDouble(SpAddress))
   {
      EEPROM_writeDouble(SpAddress, Setpoint);
   }
   if (Kp != EEPROM_readDouble(KpAddress))
   {
      EEPROM_writeDouble(KpAddress, Kp);
   }
   if (Ki != EEPROM_readDouble(KiAddress))
   {
      EEPROM_writeDouble(KiAddress, Ki);
   }
   if (Kd != EEPROM_readDouble(KdAddress))
   {
      EEPROM_writeDouble(KdAddress, Kd);
   }
   
   if (trendInterval != EEPROM_readLong(TrendAddress))
   {
      EEPROM_writeLong(TrendAddress, trendInterval);
   }
   if (tickInterval != EEPROM.read(TickAddress))
   {
      EEPROM_writeLong(TickAddress, tickInterval);
   }

}

// ************************************************
// Load parameters from EEPROM
// ************************************************
void LoadParameters()
{
  // Load from EEPROM
   Setpoint = EEPROM_readDouble(SpAddress);
   Kp = EEPROM_readDouble(KpAddress);
   Ki = EEPROM_readDouble(KiAddress);
   Kd = EEPROM_readDouble(KdAddress);
   trendInterval = EEPROM_readLong(TrendAddress);
   tickInterval = EEPROM.read(TickAddress);

   // Use defaults if EEPROM values are invalid
   if (isnan(Setpoint))
   {
     Setpoint = 25;
   }
   if (isnan(Kp))
   {
     Kp = 850;
   }
   if (isnan(Ki))
   {
     Ki = 10;
   }
   if (isnan(Kd))
   {
     Kd = 1;
   }  
   if (isnan(trendInterval) || isnan(tickInterval) || (trendInterval==0) || (tickInterval==0))
   {
     trendInterval = 1875L;
     tickInterval = 16;
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
