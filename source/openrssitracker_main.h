//-------------------------------------------------------------------
#ifndef __openrssitracker_main_H__
#define __openrssitracker_main_H__
//-------------------------------------------------------------------
 

#include <LiquidCrystal_SR3W.h>
#include <LiquidCrystal_SR2W.h>
#include <LiquidCrystal_SR.h>
#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>
#include <LCD.h>
#include <I2CIO.h>
#include <FastIO.h>
#include <Servo.h>
#include <twi.h>
#include <EEPROM.h>
#include <SimpleTimer.h>
#include <avr/wdt.h>
#include <AnalogKeyPad.h>
#include <eepromanything.h>
#include <EEPROM.h>
#include <Wire.h>
 

 
//-------------------------------------------------------------------
 
//-------------------------------------------------------------------
 
#define PIN_RSSI0	A2
#define PIN_RSSI1	A1
#define PIN_RSSI2	A0
#define PIN_RSSI3	A6
#define PIN_AVSWITCH1 2
#define PIN_AVSWITCH2 3
#define PIN_SERVO	5
#define PIN_BUZZER	4
#define PIN_VOLTAGE A7
#define PIN_MODULE4C2	9
#define PIN_MODULE3C2	8
#define PIN_MODULE2C2	7
#define PIN_MODULE1C2	6
#define PIN_MODULESC1	10
#define PIN_MODULESC3	11
#define PIN_KEYPAD	A3
#define PIN_SCL	A3
#define PIN_SDA A4


#define BTN_UP 3
#define BTN_DOWN 4
#define BTN_RIGHT 5
#define BTN_LEFT 2
#define BTN_SEL 1


#define MODULE_CLK_LOW()    digitalWrite(11, 0);
#define MODULE_CLK_HIGH()   digitalWrite(11, 1);

#define MODULE_DATA_LOW()   digitalWrite(10, 0);
#define MODULE_DATA_HIGH()  digitalWrite(10, 1);

#define MODULE_EN_LOW(x)    digitalWrite(x, 0);
#define MODULE_EN_HIGH(x)   digitalWrite(x, 1);

#define DELAY_CLOCK_US      300
#define DELAY_ENABLE_US     200
#define DELAY_CLOCKEDIN_MS  2

#define LOWSIGNAL	10
#define TRACKER_LOCKCOUNTER 50


#define STR_TRACKERNAME 		"RSSI Pro Track "
#define STR_INIT 		"Initialising"
#define STR_FORMAT 				"FORMATING EPPROM"
#define STR_PRESSKEY 			"PRESS KEY"
#define STR_CALIB_UP 			"UP:     "
#define STR_CALIB_DOWN 			"DOWN:   "
#define STR_CALIB_LEFT 			"LEFT    "
#define STR_CALIB_RIGHT 		"RIGHT:  "
#define STR_CALIB_SELECT 		"SELECT: "
#define STR_CALIB_OK			"OK "
#define STR_CALIBRATING "CALIBRATING     "
#define STR_MENU_RUN "> RUN"
#define STR_MENU_CHANNEL "> CHANNEL"
#define STR_MENU_CHANNEL_SET "> SET         "
#define STR_MENU_SETCHANNEL "SET CHANNEL"
#define STR_MENU_CHANNEL_SCAN "> SCAN        "
#define STR_MENU_SCANCHANNEL "SCAN CHANNEL      "
#define STR_MENU_CHANNEL_BROWSE "> BROWSE        "
#define STR_MENU_BROWSECHANNEL "BROWSE          "
#define STR_MENU_DIRECTION "REVERSE SERVO   "
#define STR_EXIT_TRACKING "STOP TRACKING"
#define STR_MENU_SETSPEED "SET SPEED   "
#define STR_MENU_SEEKSPEED "SEEK SPEED       "
#define STR_MENU_CENTERHOLD "CENTER HOLD   "
#define STR_MENU_KALMAN "FILTER        "
#define STR_MENU_KALMANAV "AV FILTER    "
#define STR_MENU_SETVOLTAGE "VOLTAGE          "
#define STR_MENU_SETSERVOCENTERL "CENTER L         "
#define STR_MENU_SETSERVOCENTERR "CENTER R         "
#define STR_MENU_ADVANCED "> ADVANCED"
#define STR_MENU_ADVANCED_CALIBRATE "> CALIBRATE"
#define STR_MENU_ADVANCED_SPEED "> SPEED"
#define STR_MENU_ADVANCED_CENTERHOLD "> CENTER HOLD"
#define STR_MENU_ADVANCED_KALMAN "> FILTER         "
#define STR_MENU_ADVANCED_KALMANAV "> AV FILTER   "
#define STR_MENU_ADVANCED_SERVOCENTERL "> SERVO CENTER L"
#define STR_MENU_ADVANCED_SERVOCENTERR "> SERVO CENTER R"
#define STR_MENU_ADVANCED_DIRECTION "> SERVO REVERSE"
#define STR_MENU_ADVANCED_VOLTAGE "> VOLTAGE ALARM"
#define STR_MENU_ADVANCED_SEEKSPEED "> SEEK SPEED      "

//location in which we store information in the epprom
#define EPPROM_FORMAT 0
#define EPPROM_SERVOCENTERL 10
#define EPPROM_SERVOCENTERR 15
#define EPPROM_DIRECTION 20
#define EPPROM_KALMAN 25
#define EPPROM_CHANNEL 30
#define EPPROM_SPEED 35
#define EPPROM_VOLTAGEALARM 40
#define EPPROM_BAND 45
#define EPPROM_KEYUP 100
#define EPPROM_KEYDOWN 110
#define EPPROM_KEYLEFT 120
#define EPPROM_KEYRIGHT 130
#define EPPROM_KEYSEL 140
#define EPPROM_LOWLEFT 150
#define EPPROM_LOWRIGHT 160
#define EPPROM_CENTERHOLD 170
#define EPPROM_LOWMID 180
#define EPPROM_LOWTOP 190
#define EPPROM_HIGHLEFT 200
#define EPPROM_HIGHRIGHT 210
#define EPPROM_HIGHMID 220
#define EPPROM_HIGHTOP 230
#define EPPROM_SEEKSPEED 240
#define EPPROM_KALMANAV 250


//system state switch values
#define STATE_TRACKING 1
#define STATE_MENU 2 

//main menu switch values
#define MENU_RUN 1
#define MENU_CHANNEL 2
#define MENU_CHANNEL_SET 21
#define MENU_CHANNEL_SCAN 22
#define MENU_CHANNEL_BROWSE 23
#define MENU_ADVANCED 3
#define MENU_ADVANCED_CALIBRATE 30
#define MENU_ADVANCED_SPEED 31
#define MENU_ADVANCED_CENTERHOLD 32
#define MENU_ADVANCED_KALMAN 33
#define MENU_ADVANCED_SERVOCENTERL 34
#define MENU_ADVANCED_SERVOCENTERR 35
#define MENU_ADVANCED_DIRECTION 36
#define MENU_ADVANCED_VOLTAGE 37
#define MENU_ADVANCED_KALMANAV 38


 
//-------------------------------------------------------------------
 
//===================================================================
// -> DO NOT WRITE ANYTHING BETWEEN HERE...
// 		This section is reserved for automated code generation
// 		This process tries to detect all user-created
// 		functions in main_sketch.cpp, and inject their
// 		declarations into this file.
// 		If you do not want to use this automated process,
//		simply delete the lines below, with "&MM_DECLA" text
//===================================================================
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_BEG@---------------------
float kalman_update(float measurement);
float div_kalman_update1(float measurement);
float div_kalman_update2(float measurement);
void updateChannel();
void updateVoltage();
float readVoltage();
void busyAnim(int x,int y);
void setModuleChannel(int Module,int Band, int Channel);
int setModuleChannelAll(int bandTmp, int channelTmp,int mode);
void SERIAL_ENABLE_HIGH(int Module);
void SERIAL_ENABLE_LOW(int Module);
void SERIAL_SENDBIT0();
void SERIAL_SENDBIT1();
int ReadNumber();
void eppromInit();
void menuCalibrateHigh();
void menuCalibrateLow();
void changeState(int x);
void menuWrite(char const *txt);
int checkKeyPad();
void switchMenuLoc(int loc);
void menuSelectArrow();
void menuSetVoltage();
void menuSetServoDirection();
void menuSetServoCenterR();
void menuSetServoCenterL();
void menuSetKalman();
void menuSetKalmanAV();
void menuSetCenterHold();
void menuSetSpeed();
void menuBrowseChannel();
void menuScanChannel();
void menuSetChannel();
void menuSeekSpeed();
void menu();
void lowRSSIBeep();
void rssiStats();
int getServoCenter();
void calibrate();
void track();
void loop();
void setup();
void rssiAVStats();
long readVcc();
//---- DO NOT DELETE THIS LINE -- @MM_DECLA_END@---------------------
// -> ...AND HERE. This space is reserved for automated code generation!
//===================================================================
 
 
//-------------------------------------------------------------------
#endif
//-------------------------------------------------------------------
 
 
 
 
