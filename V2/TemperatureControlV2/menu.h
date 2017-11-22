#include <MenuSystem.h>
#ifndef MENU_H_
#define MENU_H_

enum menuState      { mSTART = 0, mSELECTPID, mSP, mOP, mMODE, mTUNE_P, mTUNE_I, mTUNE_D, mSELECTSENSOR};
extern MenuSystem ms;  
extern Menu mm;  
  
extern int Menu_PIDid;

extern menuState      mState;
extern menuState      dState ;

void SetupMenu();

/*void RunMenu();*/
#endif
