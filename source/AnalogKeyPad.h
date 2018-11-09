#ifndef AnalogKeyPad_h
#define AnalogKeyPad_h

#include "Arduino.h"

#define SAMPLE_WAIT -1
#define NO_KEY 0
#define UP_KEY 3
#define DOWN_KEY 4
#define LEFT_KEY 2
#define RIGHT_KEY 5
#define SELECT_KEY 1


extern int DEFAULT_KEY_PIN; 
extern int DEFAULT_THRESHOLD;
extern int UPKEY_ARV; //that's read "analogue read value"
extern int DOWNKEY_ARV;
extern int LEFTKEY_ARV;
extern int RIGHTKEY_ARV;
extern int SELKEY_ARV;


class AnalogKeyPad
{
  public:
    AnalogKeyPad();
    int getKey();
    void setRate(int);
  private:
    int _refreshRate;
    int _keyPin;
    int _threshold;
    int _keyIn;
    int _curInput;
    int _curKey;
    int _prevInput;
    int _prevKey;
    boolean _change;
    unsigned long _oldTime;
};

#endif