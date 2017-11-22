#include "display.h"
#include "configuration.h"
#include "menu.h"
#include "control.h"
#include <MenuSystem.h>

#ifdef DISPLAY_1_8
  #include <TFT.h>               // Arduino LCD library
  TFT TFTscreen = TFT(cs, dc, rst);
#endif
#ifdef DISPLAY_2_2
  #include "Adafruit_GFX.h"
  #include "Adafruit_ILI9340.h"
  Adafruit_ILI9340 TFTscreen = Adafruit_ILI9340(cs, dc, rst);
  //Adafruit_ILI9340 tft       = Adafruit_ILI9340(cs, dc, rst);
#endif
Menu const* cp_menu ;
int SelectedAddressid;
double doublevalue;
long lastTrendTime;
long trendInterval; // trend every 30 seconds
int tickInterval; // trend every 30 seconds
char daysOfTheWeek[7][12] = {"Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday"};

int   x_pos; //position along the graph x axis
float y_pos_x; //current graph y axis position of X value
float y_pos_y1; //current graph y axis position of X value
float y_pos_y2; //current graph y axis position of X value
float y_pos_y; //current graph y axis position of Y value
float y_pos_z; //current graph y axis position of Z value

// Display variables
double lastSetpoint[NUMBER_OF_PID] ;
double lastInputValue[NUMBER_OF_PID] ;
double lastOutput[NUMBER_OF_PID] ;
boolean lastOutput_Status[NUMBER_OF_PID] ;
boolean lasttuning[NUMBER_OF_PID] ;

//----USED IN SETUP-------------------------------------------------------------------
void printAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) TFTscreen.print("0");
    TFTscreen.print(deviceAddress[i], HEX);
  }
}
void serialprintAddress(DeviceAddress deviceAddress)
{
  for (uint8_t i = 0; i < 8; i++)
  {
    // zero pad the address if necessary
    if (deviceAddress[i] < 16) TFTscreen.print("0");
    Serial.print(deviceAddress[i], HEX);
  }
}
//----SETUP DISPLAY-------------------------------------------------------------------
void SetupDisplay()
{


  
  TFTscreen.begin();
  TFTscreen.fillScreen(ILI9340_BLACK);
  TFTscreen.setRotation(DISPLAYROTATION);
  TFTscreen.setTextSize(1);
  TFTscreen.setCursor(0, 0); 
//  dState = mDISPLAY_1_15;


lastTrendTime = 0;
trendInterval = 1875L; // trend every 30 seconds
tickInterval = 16; // trend every 30 seconds

x_pos =0; //position along the graph x axis
   for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
    {
        lastSetpoint[PIDid] = 0;
        lastInputValue[PIDid] = 0;
        lastOutput[PIDid] = 0;
        lastOutput_Status[PIDid] = false;
        lasttuning[PIDid] = false;
      
    }


}

//----DISPLAY MENU-------------------------------------------------------------------
void DisplayMenu()
{
  TFTscreen.setCursor(MENU_LOCATION_X, MENU_LOCATION_Y); 
  cp_menu = ms.get_current_menu();
  TFTscreen.print(cp_menu->get_name());
  TFTscreen.setCursor(MENU_LOCATION_X, MENU_LOCATION_Y+8); 
  TFTscreen.print(cp_menu->get_selected()->get_name());

  TFTscreen.setCursor(MENU_LOCATION_X+80, MENU_LOCATION_Y); 
switch (mState)
     {
        case mSTART:
         
    TFTscreen.print(now.year());
    TFTscreen.print('/');
    TFTscreen.print(now.month());
    TFTscreen.print('/');
    TFTscreen.print(now.day());
    TFTscreen.print(" ");


    TFTscreen.print(now.hour());
    TFTscreen.print(':');
    TFTscreen.print(now.minute());
    TFTscreen.print(':');
    TFTscreen.print(now.second());
    TFTscreen.println("");
    TFTscreen.setCursor(MENU_LOCATION_X+80, MENU_LOCATION_Y+8); 
    TFTscreen.print(daysOfTheWeek[now.dayOfTheWeek()]);
         break;
        case mSP:    
         TFTscreen.print(doublevalue);
        break;
      case mOP:    
         TFTscreen.print(doublevalue);
        break;
      case mTUNE_P:
               TFTscreen.print(doublevalue);
        break;
      case mTUNE_I:        
               TFTscreen.print(doublevalue);
        break;
      case mTUNE_D:        
               TFTscreen.print(doublevalue);
        break;
      case mSELECTSENSOR:        
                printAddress(ThermometerAddress[SelectedAddressid]);
                TFTscreen.setCursor(MENU_LOCATION_X+80, MENU_LOCATION_Y+8); 
                TFTscreen.print(Temperature[SelectedAddressid]);
        break;        
     }
}


//----DISPLAY LABELS-------------------------------------------------------------------
void UpdateDisplayLabels()
{
for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
   {
  
  TFTscreen.setTextSize(TEXT_SIZE);
   TFTscreen.setTextColor(COLOR_WHITE,COLOR_BLACK);
   TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid, 0);
   TFTscreen.print("SP");
   TFTscreen.setTextColor(COLOR_GREEN,COLOR_BLACK);
   TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid, 8);
   TFTscreen.print("PV");   
   TFTscreen.setTextColor(COLOR_BLUE,COLOR_BLACK);
   TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid, 16);
   TFTscreen.print("OP"); 
   }  
}
//----DISPLAY MODE-------------------------------------------------------------------
void UpdateDisplayMode()
{
  

    
  for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
   {
            TFTscreen.setTextColor(COLOR_WHITE,COLOR_BLACK);
            TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid+54, 0);
            switch (opState[PIDid])
               {
               case oSTART:
                     TFTscreen.print("STRT");
                  break;
               case oOFF:
                     TFTscreen.print("OFF ");
                  break;
               case oRUN:
                     TFTscreen.print("RUN ");               
                  break;
               case oAUTO:
                     TFTscreen.print("AUTO");               
                  break;
               case oMAN:
                     TFTscreen.print("MAN ");               
                  break;
               }
   }            
}

//----DISPLAY VALUES-------------------------------------------------------------------
void UpdateDisplayValues()
{
   for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
    {
  
  float pct = map(PIDData[PIDid]->Output, 0, PIDData[PIDid]->WindowSize, 0, 10000);



  if (!(lastInputValue[PIDid] == PIDData[PIDid]->Input)){    
        TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid+14, 0);
        TFTscreen.setTextColor(COLOR_WHITE,COLOR_BLACK);    
        TFTscreen.print(PIDData[PIDid]->Setpoint);
        lastInputValue[PIDid] = PIDData[PIDid]->Input;
        TFTscreen.setTextColor(COLOR_GREEN,COLOR_BLACK);
        TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid+14, 8);
        TFTscreen.print(PIDData[PIDid]->Input);TFTscreen.print("  ");
  } 

  if (!(lastOutput_Status[PIDid] == Output_Status[PIDid]) || !(lastOutput[PIDid] == PIDData[PIDid]->Output)){
  lastOutput_Status[PIDid] = Output_Status[PIDid];
  lastOutput[PIDid] = PIDData[PIDid]->Output;
   TFTscreen.setCursor(CONTROLLER_DISPLAY_WIDTH*PIDid+14, 16);
    if (Output_Status[PIDid])
      {
        TFTscreen.setTextColor(COLOR_BLACK,COLOR_BLUE);
      }
      else
      {
        TFTscreen.setTextColor(COLOR_BLUE,COLOR_BLACK);
      }

     TFTscreen.print(pct/100);TFTscreen.setTextColor(COLOR_BLUE,COLOR_BLACK);TFTscreen.print("  ");

  }             
   /* TFTscreen.setTextColor(COLOR_WHITE,COLOR_BLACK);
    TFTscreen.setCursor(NUMBER_OF_PID*CONTROLLER_DISPLAY_WIDTH-(160-70), 8);
    TFTscreen.print(Ambient);
    TFTscreen.setCursor(NUMBER_OF_PID*CONTROLLER_DISPLAY_WIDTH-(160-124), 8);
    TFTscreen.print(Element);*/ 

}
}
//----DISPLAY TREND VALUES-------------------------------------------------------------------
void UpdateTrends()
{
   if ((millis() - lastTrendTime) > trendInterval)  
   {
     for(int PIDid=0; PIDid<NUMBER_OF_PID; PIDid++)
     {
        float pct = map(PIDData[PIDid]->Output, 0, PIDData[PIDid]->WindowSize, 0, 1000);

        y_pos_y  = map(PIDData[PIDid]->Input*10, 100, 300, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, CONTROLLER_DISPLAY_HEIGHT);
        y_pos_y1 = map(Element*10, 100, 300, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, CONTROLLER_DISPLAY_HEIGHT);
        y_pos_y2 = map(Ambient*10, 100, 300, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, CONTROLLER_DISPLAY_HEIGHT);
        y_pos_x  = map(PIDData[PIDid]->Setpoint*10, 100, 300, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, CONTROLLER_DISPLAY_HEIGHT);
        y_pos_z  = map(PIDData[PIDid]->Output, 0, PIDData[PIDid]->WindowSize, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, CONTROLLER_DISPLAY_HEIGHT);

        
      lastTrendTime = millis();
      x_pos = x_pos + 1;
      if (x_pos > TREND_DISPLAY_WIDTH-1) {x_pos = 0;}
       TFTscreen.fillRect(x_pos, CONTROLLER_DISPLAY_HEIGHT, 5, TREND_DISPLAY_HEIGHT, COLOR_BLACK);
       
      if (((x_pos-1)%tickInterval)==0) {  
          for (int i=10; i <= 30; i++){ // grid for every degree from 10 to 30C
            int pos = map(i, 10, 30, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, CONTROLLER_DISPLAY_HEIGHT);
            TFTscreen.drawPixel(x_pos,pos , COLOR_YELLOW); //plot single point 
          } 
      }


      TFTscreen.drawPixel(x_pos, y_pos_x, COLOR_WHITE); //plot single point 
   
      TFTscreen.drawPixel(x_pos, CONTROLLER_DISPLAY_HEIGHT, COLOR_WHITE); 
      TFTscreen.drawPixel(x_pos, CONTROLLER_DISPLAY_HEIGHT+TREND_DISPLAY_HEIGHT, COLOR_WHITE);
      TFTscreen.drawPixel(x_pos, y_pos_y, COLOR_GREEN);
      TFTscreen.drawPixel(x_pos, y_pos_y1, 0x3DEF);
      TFTscreen.drawPixel(x_pos, y_pos_y2, 0x318c);
  
      if (Output_Status)
      {
        TFTscreen.drawPixel(x_pos, y_pos_z, COLOR_RED);
      }
      else
      {
        TFTscreen.drawPixel(x_pos, y_pos_z, COLOR_BLUE);
      }  
    }
  }
}
//----DISPLAY TREND DIV-------------------------------------------------------------------
void UpdateDisplayDiv()
{
            TFTscreen.setTextColor(COLOR_WHITE,COLOR_BLACK);
            TFTscreen.setCursor(DISPLAY_DIV_X, DISPLAY_DIV_Y);



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

        TFTscreen.setCursor(DISPLAY_CTRLNO_X, DISPLAY_DIV_Y);            
        TFTscreen.print("No: "); 
        TFTscreen.print(Menu_PIDid);
}



