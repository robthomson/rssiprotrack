#include "Arduino.h"
#include "AnalogKeyPad.h"




AnalogKeyPad::AnalogKeyPad()
{	
  _refreshRate = 5;
  _keyPin = DEFAULT_KEY_PIN;
  _threshold = DEFAULT_THRESHOLD;
  _keyIn = NO_KEY;
  _curInput = NO_KEY;
  _curKey = NO_KEY;
  _prevInput = NO_KEY;
  _prevKey = NO_KEY;
  _oldTime = 0;
}

int AnalogKeyPad::getKey()
{
 if (millis() > _oldTime + _refreshRate)
 {
    _prevInput = _curInput;
    _curInput = analogRead(_keyPin);
  
    if (_curInput == _prevInput)
    {
      _change = false;
      _curKey = _prevKey;
    }
    else
    {
      _change = true;
      _prevKey = _curKey;
  
      if (_curInput > UPKEY_ARV - _threshold && _curInput < UPKEY_ARV + _threshold ) _curKey = UP_KEY;
      else if (_curInput > DOWNKEY_ARV - _threshold && _curInput < DOWNKEY_ARV + _threshold ) _curKey = DOWN_KEY;
      else if (_curInput > RIGHTKEY_ARV - _threshold && _curInput < RIGHTKEY_ARV + _threshold ) _curKey = RIGHT_KEY;
      else if (_curInput > LEFTKEY_ARV - _threshold && _curInput < LEFTKEY_ARV + _threshold ) _curKey = LEFT_KEY;
      else if (_curInput > SELKEY_ARV - _threshold && _curInput < SELKEY_ARV + _threshold ) _curKey = SELECT_KEY;
      else _curKey = NO_KEY;
   }
   
   if (_change) return _curKey; else return SAMPLE_WAIT;
   _oldTime = millis();
 }
 return 1;
}

void AnalogKeyPad::setRate(int rate)
{
  _refreshRate = rate;
}