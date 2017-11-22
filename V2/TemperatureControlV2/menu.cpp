#include <MenuSystem.h>

#include "menu.h"
#include "configuration.h"
#include "display.h"
#include "io.h"
#include "control.h"
#include "configuration.h"

int Menu_PIDid;

MenuSystem ms;  
Menu mm              ("       ");  
MenuItem mi_start    ("       ");
MenuItem mi_pidid    ("CTRL No");
Menu     mu_mode     ("MODE   ");  
MenuItem mi_mode_off ("OFF    ");  
MenuItem mi_mode_run ("RUN    ");  
MenuItem mi_mode_auto("AUTO   ");  
MenuItem mi_mode_man ("MAN    ");  

Menu     mu_spop     ("SP OP  "); 
MenuItem mi_spop_change ("CHANGE ");

Menu     mu_tune     ("TUNE   "); 
MenuItem mi_tune_sensor  ("SENSOR "); 
MenuItem mi_tune_kp  ("Kp     "); 
MenuItem mi_tune_ki  ("Ki     "); 
MenuItem mi_tune_kd  ("Kd     "); 

Menu     mu_params             ("PARAMS "); 
Menu     mu_params_display     ("X Axis ");
MenuItem mi_params_display_2mins("2 Mins ");      //mDISPLAY_1_15:                     
MenuItem mi_params_display_5mins("5 Mins ");      //mDISPLAY_2_15:                       
MenuItem mi_params_display_10mins("10 Mins");     //mDISPLAY_4_15:
MenuItem mi_params_display_15mins("15 Mins");     //mDISPLAY_6_10:
MenuItem mi_params_display_30mins("30 Mins");     //mDISPLAY_12_25:
MenuItem mi_params_display_60mins("1 Hour ");     //mDISPLAY_23_13:
MenuItem mi_params_display_120mins("2 Hours");    //mDISPLAY_45_20:
MenuItem mi_params_display_180mins("3 Hours");    //mDISPLAY_68_13:
MenuItem mi_params_display_240mins("4 Hours");    //mDISPLAY_90_10:
MenuItem mi_params_display_300mins("5 Hours");    //mDISPLAY_113_16:
MenuItem mi_params_display_360mins("6 Hours");    //mDISPLAY_135_13:
MenuItem mi_params_display_480mins("8 Hours");    //mDISPLAY_180_10:
MenuItem mi_params_display_720mins("12 Hrs ");    //mDISPLAY_270_13:
MenuItem mi_params_display_1440mins("24 Hrs ");   //mDISPLAY_540_13:
MenuItem mi_params_display_2880mins("48 Hrs ");   //mDISPLAY_1080_13:      
       
MenuItem mu_params_restart     ("RESTART");

MenuItem mi_return   ("RETURN "); 

menuState      mState;
//menuState      dState ;

// MODE EVENT ACTIONS

void on_menu_pidid (MenuItem* pMenuItem)  
{
  Menu_PIDid++;
  if(Menu_PIDid >=NUMBER_OF_PID) {Menu_PIDid=0;}
  //ms.reset();
}
void on_menu_mode_off(MenuItem* pMenuItem)  
{
  opState[Menu_PIDid] = oOFF;
  ms.reset();
}
void on_menu_mode_run(MenuItem* pMenuItem)  
{
  opState[Menu_PIDid] = oRUN;
  ms.reset();
}
void on_menu_mode_auto(MenuItem* pMenuItem)  
{
  opState[Menu_PIDid] = oAUTO;
  ms.reset();
}

void on_menu_mode_man(MenuItem* pMenuItem)  
{
  opState[Menu_PIDid] = oMAN;
  ms.reset();
}


void on_menu_spop_change(MenuItem* pMenuItem)  
{
 if (opState[Menu_PIDid] == oAUTO) {mState = mSP; doublevalue = PIDData[Menu_PIDid]->Setpoint;}
 if (opState[Menu_PIDid] == oRUN) {mState = mSP; doublevalue = PIDData[Menu_PIDid]->Setpoint;}
 if (opState[Menu_PIDid] == oMAN) {mState = mOP; doublevalue = map(PIDData[Menu_PIDid]->Output, 0, PIDData[Menu_PIDid]->WindowSize, 0, 1000)/10;}
}



void on_menu_tune_sensor(MenuItem* pMenuItem)  
{
  mState = mSELECTSENSOR; SelectedAddressid = 0;
}
void on_menu_tune_kp(MenuItem* pMenuItem)  
{
  mState = mTUNE_P; doublevalue = PIDData[Menu_PIDid]->Kp;
}
void on_menu_tune_ki(MenuItem* pMenuItem)  
{
   mState = mTUNE_I; doublevalue = PIDData[Menu_PIDid]->Ki;
}

void on_menu_tune_kd(MenuItem* pMenuItem)  
{
 mState = mTUNE_D; doublevalue = PIDData[Menu_PIDid]->Kd;
}

void on_menu_set_return(MenuItem* pMenuItem)  
{
  ms.back();
  Serial.println("Return");
}

void on_menu_params_restart(MenuItem* pMenuItem)  
{
  x_pos = 0;
  ms.reset();
}
void on_menu_params_display_2mins(MenuItem* pMenuItem){trendInterval = 750L;  tickInterval = 20; ms.reset();}
void on_menu_params_display_5mins(MenuItem* pMenuItem){trendInterval = 1875L;  tickInterval = 16; ms.reset();}
void on_menu_params_display_10mins(MenuItem* pMenuItem){trendInterval = 3750L;  tickInterval = 16; ms.reset();}
void on_menu_params_display_15mins(MenuItem* pMenuItem){;trendInterval = 5625L;  tickInterval = 11; ms.reset();}
void on_menu_params_display_30mins(MenuItem* pMenuItem){trendInterval = 11250L;  tickInterval = 27; ms.reset();}
void on_menu_params_display_60mins(MenuItem* pMenuItem){trendInterval = 22500L;  tickInterval = 13; ms.reset();}
void on_menu_params_display_120mins(MenuItem* pMenuItem){trendInterval = 45000L;  tickInterval = 20; ms.reset();}
void on_menu_params_display_180mins(MenuItem* pMenuItem){trendInterval = 67500L;  tickInterval = 10; ms.reset();}
void on_menu_params_display_240mins(MenuItem* pMenuItem){trendInterval = 90000L;  tickInterval = 10; ms.reset();}
void on_menu_params_display_300mins(MenuItem* pMenuItem){trendInterval = 112500L;  tickInterval = 16; ms.reset();}
void on_menu_params_display_360mins(MenuItem* pMenuItem){trendInterval = 135000L;  tickInterval = 13; ms.reset();}
void on_menu_params_display_480mins(MenuItem* pMenuItem){trendInterval = 180000L;  tickInterval = 10; ms.reset();}
void on_menu_params_display_720mins(MenuItem* pMenuItem){trendInterval = 270000L;  tickInterval = 13; ms.reset();}
void on_menu_params_display_1440mins(MenuItem* pMenuItem){trendInterval = 540000L;  tickInterval = 13; ms.reset();}
void on_menu_params_display_2880mins(MenuItem* pMenuItem){trendInterval = 1080000L;  tickInterval = 13; ms.reset();}


void SetupMenu()
{
  Menu_PIDid = 0;
  mState = mSTART;
  ms.set_root_menu(&mm);
    mm.add_item(&mi_start, NULL);
    mm.add_item(&mi_pidid, &on_menu_pidid);
    mm.add_menu(&mu_mode);
      mu_mode.add_item(&mi_mode_off, &on_menu_mode_off);
      mu_mode.add_item(&mi_mode_run, &on_menu_mode_run);
      mu_mode.add_item(&mi_mode_auto, &on_menu_mode_auto);
      mu_mode.add_item(&mi_mode_man, &on_menu_mode_man);
      mu_mode.add_item(&mi_return, &on_menu_set_return);
    mm.add_menu(&mu_spop);
      mu_spop.add_item(&mi_spop_change, &on_menu_spop_change);
      mu_spop.add_item(&mi_return, &on_menu_set_return);
    mm.add_menu(&mu_tune);
      mu_tune.add_item(&mi_tune_sensor, &on_menu_tune_sensor);
      mu_tune.add_item(&mi_tune_kp, &on_menu_tune_kp);
      mu_tune.add_item(&mi_tune_ki, &on_menu_tune_ki);
      mu_tune.add_item(&mi_tune_kd, &on_menu_tune_kd);
      mu_tune.add_item(&mi_return, &on_menu_set_return);
    mm.add_menu(&mu_params);
      mu_params.add_menu(&mu_params_display);
        mu_params_display.add_item(&mi_params_display_2mins,on_menu_params_display_2mins);
        mu_params_display.add_item(&mi_params_display_5mins,on_menu_params_display_5mins);
        mu_params_display.add_item(&mi_params_display_10mins,on_menu_params_display_10mins);
        mu_params_display.add_item(&mi_params_display_15mins,on_menu_params_display_15mins);
        mu_params_display.add_item(&mi_params_display_30mins,on_menu_params_display_30mins);
        mu_params_display.add_item(&mi_params_display_60mins,on_menu_params_display_60mins);
        mu_params_display.add_item(&mi_params_display_120mins,on_menu_params_display_120mins);
        mu_params_display.add_item(&mi_params_display_180mins,on_menu_params_display_180mins);
        mu_params_display.add_item(&mi_params_display_240mins,on_menu_params_display_240mins);
        mu_params_display.add_item(&mi_params_display_300mins,on_menu_params_display_300mins);
        mu_params_display.add_item(&mi_params_display_360mins,on_menu_params_display_360mins);
        mu_params_display.add_item(&mi_params_display_480mins,on_menu_params_display_480mins);
        mu_params_display.add_item(&mi_params_display_720mins,on_menu_params_display_720mins);
        mu_params_display.add_item(&mi_params_display_1440mins,on_menu_params_display_1440mins);
        mu_params_display.add_item(&mi_params_display_2880mins,on_menu_params_display_2880mins);
      mu_params.add_item(&mu_params_restart, &on_menu_params_restart);
      mu_params.add_item(&mi_return, &on_menu_set_return);          
}


