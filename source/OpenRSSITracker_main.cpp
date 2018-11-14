/*************************************************************
project: <type project name here>
author: <type your name here>
description: <type what this file does>
*************************************************************/

#include "OpenRSSITracker_main.h"

/*
  ReadAnalogVoltage

  Reads an analog input on pin 0, converts it to voltage, and prints the result to the Serial Monitor.
  Graphical representation is available using Serial Plotter (Tools > Serial Plotter menu).
  Attach the center pin of a potentiometer to pin A0, and the outside pins to +5V and ground.

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/ReadAnalogVoltage
*/




LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address board 1
//LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);  // Set the LCD I2C address board 2


//initialise hardware layer
Servo servo;


AnalogKeyPad keypad;
SimpleTimer timer;

//global vars
int localKey = 0;
int rssiLeftRaw;
int rssiRightRaw;
int rssiMidRaw;
int rssiTopRaw;
int rssiLeft;
int rssiRight;
int rssiDiff;
int rssiAVDiff;
int rssiMid;
int rssiTop;
int lowLeft;
int lowRight;
int highLeft;
int highRight;
int lowMid;
int lowTop;
int highMid;
int highTop;
int centerHold;
int avHold = 10;
int channel;
int band;
float voltagealarm;
int trackerSpeed;
int servoSpeed;
int servocenterl;
int servocenterr;
int servoDirection;
int state;
int menuLoc;
int menuRefreshLcd = 1;
int trackLoopCounter = 0;
int trackLockCounter = 0;
int statsMode = 0;
int doSeek = 0;  // start in seek mode
int doSeekTmp;
int switchVideo1 = 0;
int switchVideo2 = 0;
int avSource=1;
int isLegacyDiversity=0;



double kalman_q= 0.05;    // 0.05 do not tamper with this value unless you are looking for adventure ;-)
double kalman_r= 150;	  // default:150   this gets over-ridden by epprom retrieved value



double div1_kalman_q= 0.05;    // 0.05 do not tamper with this value unless you are looking for adventure ;-)
double div1_kalman_r= 50;	  // default:150   this gets over-ridden by epprom retrieved value


double div2_kalman_q= 0.05;    // 0.05 do not tamper with this value unless you are looking for adventure ;-)
double div2_kalman_r= 50;	  // 50  


//keypad vars
int DEFAULT_KEY_PIN = PIN_KEYPAD; 
int DEFAULT_THRESHOLD = 30;  //keybord diff threshold to trigger working
int UPKEY_ARV;
int DOWNKEY_ARV;
int LEFTKEY_ARV;
int RIGHTKEY_ARV;
int SELKEY_ARV;


/* 64 ch version */
uint16_t channelTable[] = {
  0x2A05,	0x299B,	0x2991,	0x2987,	0x291D,	0x2913,	0x2909,	0x289F,	 // BAND A
  0x2903,	0x290C,	0x2916,	0x291F,	0x2989,	0x2992,	0x299C,	0x2A05,	 // BAND B 
  0x2895,	0x288B,	0x2881,	0x2817,	0x2A0F,	0x2A19,	0x2A83,	0x2A8D,	 // BAND E
  0x2906,	0x2910,	0x291A,	0x2984,	0x298E,	0x2998,	0x2A02,	0x2A0C,  // ImmersionRC Fatshark
  0x281d, 	0x2890, 0x2902, 0x2915, 0x2987, 0x299a, 0x2a0c, 0x2a1f,  // RACE BAND
  0x259b,	0x260f,	0x2683,	0x2697,	0x270b,	0x271f,	0x2793,	0x2807,	//U BAND
  0x281b,	0x288f,	0x2903,	0x2917,	0x298b,	0x299f,	0x2a13,	0x2a87
  
};

/* 72 ch version */
uint16_t channelTableFreq[] = {
	5865, 5845, 5825, 5805, 5785, 5765, 5745, 5725,
	5733, 5752, 5771, 5790, 5809, 5828, 5847, 5866,
	5705, 5685, 5665, 5645, 5885, 5905, 5925, 5945,
	5740, 5760, 5780, 5800, 5820, 5840, 5860, 5880,
	5658, 5695, 5732, 5769, 5806, 5843, 5880, 5917,
	5333, 5373, 5413, 5453, 5493, 5533, 5573, 5613,
	5653, 5693, 5733, 5773, 5813, 5853, 5893, 5933
};




void setup() { 
	
	
	//detect lcd chipset
	Wire.begin();
	byte count = 0;	
	  for (byte i = 8; i < 120; i++)
	  {
		Wire.beginTransmission (i);
		if (Wire.endTransmission () == 0)
		  {
			if(i == 63){			//set lcd type 1
				lcd.address(0x3F);
				//fast exit
				i = 120;
			}	
			if(i == 39){			//set lcd type 2
				lcd.address(0x27);
				//fast exit
				i = 120;
			}				
		  count++;
		  } 
	  } 
	  
	
	  
	  
	
	lcd.begin(16,2);
	lcd.backlight();
	lcd.noAutoscroll();
	
	//keypad
	keypad.setRate(5);

	//epprom
	eppromInit();
	
	//Loading screen..
	lcd.setCursor(0,0); //Start at character 4 on line 0
	lcd.print(STR_TRACKERNAME);
	lcd.setCursor(0,1); //Start at character 4 on line 0
	lcd.print(STR_INIT);
	
	busyAnim(15,1);
	


	EEPROM_readAnything(EPPROM_LOWMID, lowMid);
	EEPROM_readAnything(EPPROM_LOWTOP, lowTop);
	EEPROM_readAnything(EPPROM_LOWLEFT, lowLeft);
	EEPROM_readAnything(EPPROM_LOWRIGHT, lowRight);
	EEPROM_readAnything(EPPROM_HIGHMID, highMid);
	EEPROM_readAnything(EPPROM_HIGHTOP, highTop);
	EEPROM_readAnything(EPPROM_HIGHLEFT, highLeft);
	EEPROM_readAnything(EPPROM_HIGHRIGHT, highRight);
	EEPROM_readAnything(EPPROM_CHANNEL, channel);
	EEPROM_readAnything(EPPROM_BAND, band);
	EEPROM_readAnything(EPPROM_SPEED, trackerSpeed);			
	EEPROM_readAnything(EPPROM_CENTERHOLD, centerHold);
	EEPROM_readAnything(EPPROM_VOLTAGEALARM, voltagealarm);	
	EEPROM_readAnything(EPPROM_SERVOCENTERL, servocenterl);
	EEPROM_readAnything(EPPROM_SERVOCENTERR, servocenterr);	
	EEPROM_readAnything(EPPROM_KALMAN, kalman_r);
	EEPROM_readAnything(EPPROM_KALMANAV, div1_kalman_r);
	EEPROM_readAnything(EPPROM_KALMANAV, div1_kalman_r);	
	EEPROM_readAnything(EPPROM_DIRECTION, servoDirection);
    EEPROM_readAnything(EPPROM_KEYUP, UPKEY_ARV); 
	EEPROM_readAnything(EPPROM_KEYDOWN, DOWNKEY_ARV);  
	EEPROM_readAnything(EPPROM_KEYLEFT, LEFTKEY_ARV);
	EEPROM_readAnything(EPPROM_KEYRIGHT, RIGHTKEY_ARV);
	EEPROM_readAnything(EPPROM_KEYSEL, SELKEY_ARV);	





	
	//set channels on modules
	//spi bit bash for 32ch system
	pinMode(PIN_MODULESC3, OUTPUT);
	pinMode(PIN_MODULESC1, OUTPUT);
	pinMode(PIN_MODULE1C2, OUTPUT);
	pinMode(PIN_MODULE2C2, OUTPUT);
	pinMode(PIN_MODULE3C2, OUTPUT);
	pinMode(PIN_MODULE4C2, OUTPUT);

	digitalWrite(PIN_MODULESC3, LOW);
	digitalWrite(PIN_MODULESC1, LOW);
	digitalWrite(PIN_MODULE1C2, LOW);
	digitalWrite(PIN_MODULE2C2, LOW);
	digitalWrite(PIN_MODULE3C2, LOW);
	digitalWrite(PIN_MODULE4C2, LOW);

	//attach servo
	servo.attach(PIN_SERVO);

	//enable sertial port
	Serial.begin(9600);  

	//set servo to centre
	servo.writeMicroseconds(getServoCenter());

	//set the channel on startup
	MODULE_EN_HIGH(PIN_MODULE1C2);
	MODULE_EN_HIGH(PIN_MODULE2C2);
	MODULE_EN_HIGH(PIN_MODULE3C2);
	MODULE_EN_HIGH(PIN_MODULE4C2);

	setModuleChannel(PIN_MODULE1C2, band, channel);
	setModuleChannel(PIN_MODULE2C2, band, channel);
	setModuleChannel(PIN_MODULE3C2, band, channel);
	setModuleChannel(PIN_MODULE4C2, band, channel);

	MODULE_EN_HIGH(PIN_MODULE1C2);
	MODULE_EN_HIGH(PIN_MODULE2C2);
	MODULE_EN_HIGH(PIN_MODULE3C2);
	MODULE_EN_HIGH(PIN_MODULE4C2); 	

 
	
	pinMode(PIN_AVSWITCH1, OUTPUT);
	pinMode(PIN_AVSWITCH2, OUTPUT);
	digitalWrite(PIN_AVSWITCH1, HIGH);
	digitalWrite(PIN_AVSWITCH2, LOW);
 
 	timer.setInterval(5000, updateVoltage);  //update every 30 seconds
 	timer.setInterval(10000,lowRSSIBeep);


	lcd.clear();

	//check if old or new style diversity.
	analogRead(PIN_RSSI3); // Fake read to let ADC settle
	rssiTopRaw = analogRead(PIN_RSSI3);
	if(rssiTopRaw < 10){
			isLegacyDiversity=1;
	}



	updateVoltage();
	updateChannel();


	//default load state and menu position
	state = STATE_MENU;
	menuLoc = MENU_RUN;
	
	

 
}

//used to override reset
void(* resetFunc) (void) = 0;



void loop() {
	

			switch (state){
				case STATE_TRACKING:
						track();
						break;		
				case STATE_MENU:
						menu();
						break;
			}
			timer.run();   
	


	
}

void track(){

	
		//detect if attempting to rapid cancel and reset device.
		//this is done by holding down the keypad while in 'track mode'
		analogRead(DEFAULT_KEY_PIN); //adc stabilize
		if(analogRead(DEFAULT_KEY_PIN) > 5){
		
			lcd.clear();
			delay(50);
			lcd.setCursor(0,0);
			lcd.print("RESET");
			delay(1000);
			lcd.setCursor(0,1);
			lcd.print("Release key.... ");
			delay(1000);
			while(analogRead(DEFAULT_KEY_PIN) < 2){
				resetFunc();
				break;
			}	
			
		
		}	

		//commence tracking

		analogRead(PIN_RSSI2); // Fake read to let ADC settle.
		rssiLeftRaw = analogRead(PIN_RSSI2) ;
	
		analogRead(PIN_RSSI0); // Fake read to let ADC settle.
		rssiRightRaw = analogRead(PIN_RSSI0);
	
		analogRead(PIN_RSSI1); // Fake read to let ADC settle.
		rssiMidRaw = analogRead(PIN_RSSI1);
	
		analogRead(PIN_RSSI3); // Fake read to let ADC settle
		rssiTopRaw = analogRead(PIN_RSSI3);

		//filters used to smooth out noisy rssi signals
		rssiMidRaw = div_kalman_update1(rssiMidRaw);
		rssiTopRaw = div_kalman_update2(rssiTopRaw);

		
				rssiLeft = map(rssiLeftRaw, lowLeft, highLeft, 0, 100);	
				rssiRight = map(rssiRightRaw, lowRight, highRight, 0, 100);	
				rssiMid = map(rssiMidRaw, lowMid, highMid, 0, 100);
				rssiTop = map(rssiTopRaw, lowTop, highTop, 0, 100);
	
	
				rssiLeft = constrain(rssiLeft,0,100);
				rssiRight = constrain(rssiRight,0,100);
				rssiMid = constrain(rssiMid,0,100);
				rssiTop = constrain(rssiTop,0,100);				
				
				if(isLegacyDiversity == 0){		//only do this if we are in new rssi mode
					
						int raw_avrssiDiff = rssiTop > rssiMid ? rssiTop- rssiMid : rssiMid - rssiTop;

						//if diffference large enough - then swap.
						//to be determined if the difference var would benifit from a wider value.
						if(raw_avrssiDiff > 1){  //run the switching code as > 1%
								if(rssiMid > rssiTop){  //set signal to front antenna
										digitalWrite(PIN_AVSWITCH1, HIGH);
										digitalWrite(PIN_AVSWITCH2, LOW);
										switchVideo1 = 0;	
										avSource=1;
										lcd.setCursor(10,1);
										lcd.print("RX1");	
										
								} 	
								if(rssiTop > rssiMid){ //set signal to top antenna
										digitalWrite(PIN_AVSWITCH1, LOW);
										digitalWrite(PIN_AVSWITCH2, HIGH);
										switchVideo2 = 0;
										avSource=2;
										lcd.setCursor(10,1);
										lcd.print("RX2");	
								}						
						}	
					}
					
	
				//tracking diff check
				int raw_rssiDiff = rssiLeft > rssiRight ? rssiLeft - rssiRight : rssiRight - rssiLeft;
				float average_rssi = (rssiLeft + rssiRight ) / 2.f;
				const float K_rssi_ratio = 250.f; 
				rssiDiff = (int)( (raw_rssiDiff * K_rssi_ratio) / average_rssi )  ;
				rssiDiff = kalman_update(rssiDiff);

				//servo speed is directly related to the 'difference'.
				//by using the speed, we create a simple 'acceleration' to the tracker - net result - low wobble.
				servoSpeed = abs(rssiDiff);
				servoSpeed = servoSpeed + trackerSpeed; //user speed offset
				
				
				//this lock simply stops the tracker moving too much.
				//once we have a 'centerHold' we pause tracking for X loops.
				//default value for a lock is 50.  To make the tracker more 'hot'
				//reduce the value.
				if(trackLockCounter == 0){
					if(rssiMid < 10 && rssiTop < 10){  //seem to have no rf?
						servo.writeMicroseconds(getServoCenter());
						lcd.setCursor(15,1);
						lcd.print("#");						
						trackLockCounter = TRACKER_LOCKCOUNTER;
					}else if (rssiDiff <= centerHold) {
						servo.writeMicroseconds(getServoCenter());
						lcd.setCursor(15,1);
						lcd.print("^");		
						trackLockCounter = TRACKER_LOCKCOUNTER;
						
					}			
					else if(rssiLeft > rssiRight) {
						if(servoDirection == 0){
							servo.writeMicroseconds(servocenterl -  servoSpeed);
						} else {
							servo.writeMicroseconds(servocenterl +  servoSpeed);
						}
						lcd.setCursor(15,1);
						lcd.print("<");		
					}
					else if(rssiRight > rssiLeft) {
						if(servoDirection == 0){
							servo.writeMicroseconds(servocenterr +  servoSpeed);
						} else {
							servo.writeMicroseconds(servocenterr -  servoSpeed);
						}
						lcd.setCursor(15,1);
						lcd.print(">");				
					}

				} else {
						trackLockCounter--;
						if(trackLockCounter < 0){
							trackLockCounter = 0;
						}				
				}

					
				if(trackLoopCounter == 1){
					rssiStats();
				}
	
				trackLoopCounter++;
				if(trackLoopCounter == 2500){
					trackLoopCounter = 0; //redisplay stats.
				}
				
		
				
				
}

void calibrateLow(){
	
		int loops = 50;
		int i;
		int c;	

			lcd.setCursor(0,1);
			lcd.print(STR_CALIBRATING);


			//left
			c = 0;
			lowLeft= 0;
			analogRead(PIN_RSSI2); // Fake read to let ADC settle.	
				for(i=0;i<loops;i++){
					lowLeft = lowLeft  + analogRead(PIN_RSSI2);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				lowLeft = abs(lowLeft/loops);
				

			//right
			c = 0;
			lowRight = 0;	
			analogRead(PIN_RSSI0); // Fake read to let ADC settle.	
				for(i=0;i<loops;i++){

					lowRight = lowRight  + analogRead(PIN_RSSI0);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				lowRight = abs(lowRight/loops);
				
			//mid
			c = 0;
			lowMid = 0;	
			analogRead(PIN_RSSI1); // Fake read to let ADC settle.
				for(i=0;i<loops;i++){
					lowMid = lowMid  + analogRead(PIN_RSSI1);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				lowMid = abs(lowMid/loops);				
				
				
			//top
			c = 0;
			lowTop = 0;
			analogRead(PIN_RSSI3); // Fake read to let ADC settle.
				for(i=0;i<loops;i++){
					lowTop = lowTop  + analogRead(PIN_RSSI3);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				lowTop = abs(lowTop/loops);		
				
							
				

				EEPROM_writeAnything(EPPROM_LOWLEFT, lowLeft);
				EEPROM_writeAnything(EPPROM_LOWRIGHT, lowRight);
				EEPROM_writeAnything(EPPROM_LOWMID, lowMid);
				EEPROM_writeAnything(EPPROM_LOWTOP, lowTop);
				
				/*
				//clear status line
				lcd.setCursor(0, 1);
				lcd.print("L ");	
				lcd.print(lowLeft);
				lcd.print(" ");
				lcd.print("R ");	
				lcd.print(lowRight);	
				lcd.print("              ");		
				delay(2000);				

				//clear status line
				lcd.setCursor(0, 1);
				lcd.print("F ");	
				lcd.print(lowMid);
				lcd.print(" ");
				lcd.print("T ");	
				lcd.print(lowTop);	
				lcd.print("              ");		
				delay(2000);				
				*/
				
				lcd.setCursor(0, 1);
				lcd.print("              ");

				//changeState(STATE_MENU);
				menuCalibrateHigh();

}

void calibrateHigh(){

		int loops = 50;
		int i;
		int c;
	
	
			lcd.setCursor(0,1);
			lcd.print(STR_CALIBRATING);
	

			//left
			c = 0;
			highLeft= 0;
			analogRead(PIN_RSSI2); // Fake read to let ADC settle.	
				for(i=0;i<loops;i++){
					highLeft = highLeft  + analogRead(PIN_RSSI2);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				highLeft = abs(highLeft/loops);
				

			//right
			c = 0;
			highRight = 0;	
			analogRead(PIN_RSSI0); // Fake read to let ADC settle.	
				for(i=0;i<loops;i++){

					highRight = highRight  + analogRead(PIN_RSSI0);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				highRight = abs(highRight/loops);
				
			//mid
			c = 0;
			highMid = 0;	
			analogRead(PIN_RSSI1); // Fake read to let ADC settle.
				for(i=0;i<loops;i++){
					highMid = highMid  + analogRead(PIN_RSSI1);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				highMid = abs(highMid/loops);				
				
			//top
			c = 0;
			highTop = 0;
			analogRead(PIN_RSSI3); // Fake read to let ADC settle.
				for(i=0;i<loops;i++){
					highTop = highTop  + analogRead(PIN_RSSI3);
									
					if(c == 0){
						lcd.setCursor(12,1);
						lcd.print("|");
						c = 1;
					} else {
						lcd.setCursor(12,1);
						lcd.print("-");
						c = 0;
					}
					 
					delay(50);
				}
				highTop = abs(highTop/loops);				
				

							
				//store values

				EEPROM_writeAnything(EPPROM_HIGHLEFT, highLeft);
				EEPROM_writeAnything(EPPROM_HIGHRIGHT, highRight);
				EEPROM_writeAnything(EPPROM_HIGHMID, highMid);
				EEPROM_writeAnything(EPPROM_HIGHTOP, highTop);
				
				/*
				//clear status line
				lcd.setCursor(0, 1);
				lcd.print("L ");	
				lcd.print(highLeft);
				lcd.print(" ");
				lcd.print("R ");	
				lcd.print(highRight);	
				lcd.print("              ");		
				delay(2000);				

				//clear status line
				lcd.setCursor(0, 1);
				lcd.print("F ");	
				lcd.print(highMid);
				lcd.print(" ");
				lcd.print("T ");	
				lcd.print(highTop);	
				lcd.print("              ");		
				delay(2000);				
				
								*/
								
				lcd.setCursor(0, 1);
				lcd.print("              ");


				changeState(STATE_MENU);

}


int getServoCenter(){
			
		int servocenter = abs((servocenterl + servocenterr) / 2);
		return servocenter;

}

void rssiStats(){

	if(state == STATE_TRACKING){

		lcd.setCursor(0, 1);
		lcd.print("L");
		lcd.print(rssiLeft);
		if(rssiLeft < 100){
			lcd.print(" ");
		}		
	
		lcd.setCursor(5, 1);
		lcd.print("R");
		lcd.print(rssiRight);
		if(rssiRight < 100){
			lcd.print(" ");
		}


		//lcd.setCursor(10, 1);
		//lcd.print("    ");
		//lcd.setCursor(10, 1);
		//lcd.print("FRONT");
		//lcd.print(servoSpeed);	
		//if(rssiLeft < 100){
		//	lcd.print(" ");
		//}	
	}	
}





void lowRSSIBeep(){

	if(state == STATE_TRACKING){
		if(rssiLeft < LOWSIGNAL || rssiRight < LOWSIGNAL){
				tone(PIN_BUZZER, 3000, 500);
				delay(200);
				tone(PIN_BUZZER, 3000, 500);
		} 
	}	
}

void menu(){

	localKey = checkKeyPad();
	switch (menuLoc){
		case MENU_RUN:
				menuWrite(STR_MENU_RUN);
				if(localKey == BTN_DOWN){
					switchMenuLoc(MENU_CHANNEL);
				}
				if(localKey == BTN_SEL){
					menuSelectArrow();
					lcd.setCursor(0, 1);
					lcd.print("              ");
					changeState(STATE_TRACKING);	
				}				
				break;		

		case MENU_CHANNEL:
				menuWrite(STR_MENU_CHANNEL);
				if(localKey == BTN_UP){
					switchMenuLoc(MENU_RUN);
				}
				if(localKey == BTN_DOWN){
					switchMenuLoc(MENU_ADVANCED);
				}				
				if(localKey == BTN_SEL || localKey == BTN_RIGHT){
					menuSelectArrow();
					switchMenuLoc(MENU_CHANNEL_SET);
				}			
				break;
				
					//indented for clarity as sub menu
					case MENU_CHANNEL_SET:
							menuWrite(STR_MENU_CHANNEL_SET);
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_CHANNEL_SCAN);
							}
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_CHANNEL);
							}								
							if(localKey == BTN_SEL){
							menuSelectArrow();
							menuSetChannel();
							}		
							break;	
					case MENU_CHANNEL_SCAN:
							menuWrite(STR_MENU_CHANNEL_SCAN);
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_CHANNEL_SET);
							}
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_CHANNEL_BROWSE);
							}							
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_CHANNEL);
							}								
							if(localKey == BTN_SEL){
							menuSelectArrow();
							menuScanChannel();
							}		
							break;								
					case MENU_CHANNEL_BROWSE:
							menuWrite(STR_MENU_CHANNEL_BROWSE);
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_CHANNEL_SCAN);
							}
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_CHANNEL);
							}								
							if(localKey == BTN_SEL){
							menuSelectArrow();
							menuBrowseChannel();
							}		
							break;	
		case MENU_ADVANCED:
				menuWrite(STR_MENU_ADVANCED);
				if(localKey == BTN_UP){
					switchMenuLoc(MENU_CHANNEL);
				}
				if(localKey == BTN_SEL || localKey == BTN_RIGHT){
					menuSelectArrow();
					switchMenuLoc(MENU_ADVANCED_CALIBRATE);
				}			
				break;
				
					//indented for clarity as sub menu
					case MENU_ADVANCED_CALIBRATE:
							menuWrite(STR_MENU_ADVANCED_CALIBRATE);
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_SPEED);
							}
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}								
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuCalibrateLow();
							}		
							break;	

					case MENU_ADVANCED_SPEED:
							menuWrite(STR_MENU_ADVANCED_SPEED);
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_CENTERHOLD);
							}
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_CALIBRATE);
							}								
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}	
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetSpeed();
							}								
							break;
					case MENU_ADVANCED_CENTERHOLD:
							menuWrite(STR_MENU_ADVANCED_CENTERHOLD);
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_KALMAN);
							}
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_SPEED);
							}								
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}	
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetCenterHold();
							}								
							break;									
					case MENU_ADVANCED_KALMAN:
							menuWrite(STR_MENU_ADVANCED_KALMAN);
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_SERVOCENTERL);
							}
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_CENTERHOLD);
							}							
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}	
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetKalman();
							}								
							break;		
					case MENU_ADVANCED_SERVOCENTERL:
							menuWrite(STR_MENU_ADVANCED_SERVOCENTERL);
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_KALMAN);
							}
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_SERVOCENTERR);
							}
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetServoCenterL();
							}								
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}								
							break;					
					case MENU_ADVANCED_SERVOCENTERR:
							menuWrite(STR_MENU_ADVANCED_SERVOCENTERR);
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_SERVOCENTERL);
							}
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_DIRECTION);
							}
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetServoCenterR();
							}								
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}								
							break;	
								
					case MENU_ADVANCED_DIRECTION:
							menuWrite(STR_MENU_ADVANCED_DIRECTION);
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_SERVOCENTERR);
							}
							if(localKey == BTN_DOWN){
								switchMenuLoc(MENU_ADVANCED_VOLTAGE);
							}
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetServoDirection();
							}								
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}								
							break;		
					case MENU_ADVANCED_VOLTAGE:
							menuWrite(STR_MENU_ADVANCED_VOLTAGE);
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_DIRECTION);
							}
							if(localKey == BTN_DOWN){
									if(isLegacyDiversity == 0){  //we dont show last menu if legacy mode enabled
										switchMenuLoc(MENU_ADVANCED_KALMANAV);
									}	
							}
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetVoltage();
							}								
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}								
							break;	
							
					case MENU_ADVANCED_KALMANAV:
							menuWrite(STR_MENU_ADVANCED_KALMANAV);
							//if(localKey == BTN_DOWN){
							//	switchMenuLoc(MENU_ADVANCED_SERVOCENTERL);
							//}
							if(localKey == BTN_UP){
								switchMenuLoc(MENU_ADVANCED_VOLTAGE);
							}							
							if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
							}	
							if(localKey == BTN_SEL){
								menuSelectArrow();
								menuSetKalmanAV();
							}								
							break;				
													
		default:
				break;
	}


}

void menuCalibrateLow(){
		
		lcd.setCursor(0,1);
		lcd.print("VTX OFF      [] ");
		lcd.setCursor(13,1);
	
	
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();	


				if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
				}	
	
				if(localKey == BTN_SEL){
						delay(500);
						calibrateLow();
						thisLoop=0;
				}
				
		}		
}

void menuCalibrateHigh(){
		
		lcd.setCursor(0,1);
		lcd.print("VTX ON      [] ");
		lcd.setCursor(13,1);
	
	
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();	


				if(localKey == BTN_LEFT){
								switchMenuLoc(MENU_ADVANCED);
				}	
	
				if(localKey == BTN_SEL){
						delay(500);
						calibrateHigh();
						thisLoop=0;
				}
				
		}		
}


void menuSetChannel(){


		EEPROM_readAnything(EPPROM_CHANNEL, channel);
		EEPROM_readAnything(EPPROM_BAND, band);
		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_SETCHANNEL);
		lcd.setCursor(13,1);
		lcd.print((band + 1));			
		lcd.print(channel + 1);		
	
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();	
				if(localKey == BTN_UP){
					delay(500);

					band++;

					//if(band > 3){
					//if(band > 4){
					if(band > 6){
						band = 0;
					}
					
					setModuleChannelAll(band,channel,1);
					
					lcd.setCursor(13,1);
					lcd.print((band + 1));
					lcd.print((channel + 1));	

				}	
				
				if(localKey == BTN_DOWN){
					delay(500);

					band--;

					if(band < 0){
						//band = 3;
						//band = 4;
						band = 6;
					}
					
					setModuleChannelAll(band,channel,1);
					
					lcd.setCursor(13,1);
					lcd.print((band + 1));
					lcd.print((channel + 1));				


				}			

				if(localKey == BTN_RIGHT){
					delay(500);
					channel++;
					
					if(channel > 7){
						channel = 0;
					}
					
					setModuleChannelAll(band,channel,1);
					
					lcd.setCursor(13,1);
					lcd.print((band + 1));
					lcd.print((channel + 1));					
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					channel--;

					if(channel < 0){
						channel = 7;
					}
					
					setModuleChannelAll(band,channel,1);
					
					lcd.setCursor(13,1);
					lcd.print((band + 1));
					lcd.print(channel + 1);					
				}	
	
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_CHANNEL, channel);	
						EEPROM_writeAnything(EPPROM_BAND, band);	
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}

void menuScanChannel(){

		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_SCANCHANNEL);

		
		int doScan = 0;
		lcd.setCursor(15,1);
		lcd.print("N");		
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT || localKey == BTN_LEFT){
					delay(500);
					
					if(doScan == 0){
						doScan = 1;
					} else {
						doScan = 0;
					}
					
					lcd.setCursor(15,1);
					if(doScan == 0){
						lcd.print("N");
					}else{
						lcd.print("Y");
					}
				}					
				if(localKey == BTN_SEL){
						delay(500);
	
						if(doScan == 0){
							busyAnim(15,1);
							thisLoop = 0; //break the loop
							changeState(STATE_MENU);
						} else {

							EEPROM_readAnything(EPPROM_CHANNEL, channel);
							EEPROM_readAnything(EPPROM_BAND, band);

							lcd.setCursor(14,1);
							lcd.print("    ");

							int found = 0;
							int scanCount = 0;
							int loc = 0;
							int bestRSSI = 0;
							int bestChannel = channel;
							int bestBand = band;
							int thisRSSI = 0;

								
							while(found == 0){
										

										channel++;
										if(channel > 7){
											channel = 0;
											band++;
											//if(band > 3){ //32 ch
											if(band > 4){ 
												band = 0;
											} 
										}


										 thisRSSI = setModuleChannelAll(band,channel,2);
										 if(thisRSSI > bestRSSI){
												bestRSSI = thisRSSI;
												bestChannel = channel;
												bestBand = band;
										 }

										if(scanCount >= 41){
												found = 1;
										}
					
										scanCount++;

				

										lcd.setCursor(14,1);
										if(loc == 0){
											lcd.print("-");
											loc = 1;
										} else {
											lcd.print("|");
											loc = 0;
										}
										

								}
								lcd.setCursor(13,1);
								lcd.print(bestBand + 1);	
								lcd.print(bestChannel + 1);	

								band = bestBand;
								channel = bestChannel;	
								setModuleChannelAll(band,channel,1);
								
								delay(2000);
			

								EEPROM_writeAnything(EPPROM_CHANNEL, channel);	
								EEPROM_writeAnything(EPPROM_BAND, band);	
								busyAnim(15,1);
								thisLoop = 0; //break the loop
								changeState(STATE_MENU);



						}
				}
				
		}		
}


void menuBrowseChannel(){


		EEPROM_readAnything(EPPROM_CHANNEL, channel);
		EEPROM_readAnything(EPPROM_BAND, band);
		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_BROWSECHANNEL);
		lcd.setCursor(13,1);
		lcd.print((band + 1));			
		lcd.print(channel + 1);		
	
		int thisLoop = 1;


		while(thisLoop == 1){
		
			localKey = checkKeyPad();	
	



				if(localKey == BTN_RIGHT){
					delay(500);

			
					int found = 0;
					
					while(found == 0){
							lcd.setCursor(13,1);

							channel++;
							if(channel > 7){
								channel = 0;
								band++;
								//if(band > 3){ //32ch
								if(band > 4){
									band = 0;
								} 
							}

							lcd.print(band + 1);
							lcd.print(channel + 1);	
		
							found = setModuleChannelAll(band,channel,1);

					}


				}	
				
				if(localKey == BTN_LEFT){
					delay(500);

			
					int found = 0;
					
					while(found == 0){
							lcd.setCursor(13,1);

							channel--;
							if(channel < 0){
								channel = 7;
								band--;
								if(band < 0){
									//band = 3;
									band = 4;
								} 
							}

							lcd.print(band + 1);
							lcd.print(channel + 1);	
		
							found = setModuleChannelAll(band,channel,1);

					}


				}			


				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_CHANNEL, channel);	
						EEPROM_writeAnything(EPPROM_BAND, band);	
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}




void menuSetSpeed(){


		EEPROM_readAnything(EPPROM_SPEED, trackerSpeed);		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_SETSPEED);
		lcd.setCursor(12,1);
		lcd.print(trackerSpeed);
		if(trackerSpeed < 10){
			lcd.print(" ");
		}		
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					trackerSpeed++;
					if( trackerSpeed > 45){ trackerSpeed = 45;}	
					
					
					lcd.setCursor(12,1);
					lcd.print(trackerSpeed);
					if(trackerSpeed < 10){
						lcd.print(" ");
					}
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					trackerSpeed--;
					if( trackerSpeed < -45){ trackerSpeed = -45;}	

	
					lcd.setCursor(12,1);
					lcd.print(trackerSpeed);	
					if(trackerSpeed < 10){
						lcd.print(" ");
					}	
				}	
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_SPEED, trackerSpeed);	
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}



void menuSetCenterHold(){


		EEPROM_readAnything(EPPROM_CENTERHOLD, centerHold);		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_CENTERHOLD);
		lcd.setCursor(12,1);
		lcd.print(centerHold);
		if(trackerSpeed < 10){
			lcd.print(" ");
		}		
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					centerHold++;
					if( centerHold> 30){ centerHold = 30;}	
					
					
					lcd.setCursor(12,1);
					lcd.print(centerHold);
					if(centerHold < 10){
						lcd.print(" ");
					}
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					centerHold--;
					if( centerHold < 0){ centerHold = 0;}	

	
					lcd.setCursor(12,1);
					lcd.print(centerHold);	
					if(centerHold < 10){
						lcd.print(" ");
					}	
				}	
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_CENTERHOLD, centerHold);	
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}



void menuSetKalman(){

		int KalmanTmp;


		EEPROM_readAnything(EPPROM_KALMAN, KalmanTmp);	

		KalmanTmp = KalmanTmp;
	
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_KALMAN);
		lcd.setCursor(12,1);
		lcd.print(KalmanTmp);
		if(KalmanTmp < 10){
			lcd.print(" ");
		}		
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					KalmanTmp = KalmanTmp + 1;
					if( KalmanTmp > 200){ KalmanTmp = 200;}	
					
					lcd.setCursor(12,1);
					lcd.print(KalmanTmp);
					if(kalman_r < 10){
						lcd.print(" ");
					}
					if(KalmanTmp < 100){
						lcd.print(" ");
					}					
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					KalmanTmp = KalmanTmp - 1;
					if( KalmanTmp < 1){ KalmanTmp = 1;}	

					
					lcd.setCursor(12,1);
					lcd.print(KalmanTmp);	
					if(KalmanTmp < 10){
						lcd.print(" ");
					}	
					if(KalmanTmp < 100){
						lcd.print(" ");
					}	
				}	
				if(localKey == BTN_UP){
					delay(500);
					KalmanTmp = KalmanTmp + 10;
					if( KalmanTmp > 200){ KalmanTmp = 200;}	
					
					lcd.setCursor(12,1);
					lcd.print(KalmanTmp);
					if(kalman_r < 10){
						lcd.print(" ");
					}
					if(KalmanTmp < 100){
						lcd.print(" ");
					}					
				}	
				if(localKey == BTN_DOWN){
					delay(500);
					KalmanTmp = KalmanTmp - 10;
					if( KalmanTmp > 200){ KalmanTmp = 200;}	
					
					lcd.setCursor(12,1);
					lcd.print(KalmanTmp);
					if(kalman_r < 10){
						lcd.print(" ");
					}
					if(KalmanTmp < 100){
						lcd.print(" ");
					}					
				}					
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_KALMAN, KalmanTmp);	
						kalman_r = KalmanTmp;
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}


void menuSetKalmanAV(){

		int KalmanAVTmp;


		EEPROM_readAnything(EPPROM_KALMANAV, KalmanAVTmp);	

		KalmanAVTmp = KalmanAVTmp;
	
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_KALMANAV);
		lcd.setCursor(12,1);
		lcd.print(KalmanAVTmp);
		if(KalmanAVTmp < 10){
			lcd.print(" ");
		}		
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					KalmanAVTmp = KalmanAVTmp + 1;
					if( KalmanAVTmp > 200){ KalmanAVTmp = 200;}	
					
					lcd.setCursor(12,1);
					lcd.print(KalmanAVTmp);
					if(KalmanAVTmp < 10){
						lcd.print(" ");
					}
					if(KalmanAVTmp < 100){
						lcd.print(" ");
					}					
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					KalmanAVTmp = KalmanAVTmp - 1;
					if( KalmanAVTmp < 1){ KalmanAVTmp = 1;}	

					
					lcd.setCursor(12,1);
					lcd.print(KalmanAVTmp);	
					if(KalmanAVTmp < 10){
						lcd.print(" ");
					}	
					if(KalmanAVTmp < 100){
						lcd.print(" ");
					}	
				}	
				if(localKey == BTN_UP){
					delay(500);
					KalmanAVTmp = KalmanAVTmp + 10;
					if( KalmanAVTmp > 200){ KalmanAVTmp = 200;}	
					
					lcd.setCursor(12,1);
					lcd.print(KalmanAVTmp);
					if(KalmanAVTmp < 10){
						lcd.print(" ");
					}
					if(KalmanAVTmp < 100){
						lcd.print(" ");
					}					
				}	
				if(localKey == BTN_DOWN){
					delay(500);
					KalmanAVTmp = KalmanAVTmp - 10;
					if( KalmanAVTmp > 200){ KalmanAVTmp = 200;}	
					
					lcd.setCursor(12,1);
					lcd.print(KalmanAVTmp);
					if(KalmanAVTmp < 10){
						lcd.print(" ");
					}
					if(KalmanAVTmp < 100){
						lcd.print(" ");
					}					
				}					
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_KALMANAV, KalmanAVTmp);	
						div1_kalman_r = KalmanAVTmp;
						div2_kalman_r = KalmanAVTmp;
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}



void menuSetServoCenterL(){


		EEPROM_readAnything(EPPROM_SERVOCENTERL, servocenterl);		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_SETSERVOCENTERL);
		lcd.setCursor(10,1);
		lcd.print(servocenterl);
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					servocenterl++;
					if( servocenterl > 1600){ servocenterl = 1600;}	

					servo.write(servocenterl);
					
					lcd.setCursor(10,1);
					lcd.print(servocenterl);
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					servocenterl--;
					if( servocenterl < 1400){ servocenterl = 1400;}

					servo.write(servocenterl);	

					lcd.setCursor(10,1);
					lcd.print(servocenterl);	
				}	
				if(localKey == BTN_UP){
					delay(500);
					servocenterl = servocenterl + 10;
					if( servocenterl > 1600){ servocenterl= 1600;}	

					servo.write(servocenterl);
					
					lcd.setCursor(10,1);
					lcd.print(servocenterl);
				}
				if(localKey == BTN_DOWN){
					delay(500);
					servocenterl = servocenterl - 10;
					if( servocenterl < 1400){ servocenterl = 1400;}

					servo.write(servocenterl);	

					lcd.setCursor(10,1);
					lcd.print(servocenterl);	
				}					
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_SERVOCENTERL, servocenterl);	
						busyAnim(15,1);
						servo.write(getServoCenter());	
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}

void menuSetServoCenterR(){


		EEPROM_readAnything(EPPROM_SERVOCENTERR, servocenterr);		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_SETSERVOCENTERR);
		lcd.setCursor(10,1);
		lcd.print(servocenterr);
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					servocenterr++;
					if( servocenterr > 1600){ servocenterr = 1600;}	

					servo.write(servocenterr);
					
					lcd.setCursor(10,1);
					lcd.print(servocenterr);
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					servocenterr--;
					if( servocenterr < 1400){ servocenterr = 1400;}

					servo.write(servocenterr);	

					lcd.setCursor(10,1);
					lcd.print(servocenterr);	
				}	
				if(localKey == BTN_UP){
					delay(500);
					servocenterr = servocenterr + 10;
					if( servocenterr > 1600){ servocenterr= 1600;}	

					servo.write(servocenterr);
					
					lcd.setCursor(10,1);
					lcd.print(servocenterr);
				}
				if(localKey == BTN_DOWN){
					delay(500);
					servocenterr = servocenterr - 10;
					if( servocenterr < 1400){ servocenterr = 1400;}

					servo.write(servocenterr);	

					lcd.setCursor(10,1);
					lcd.print(servocenterr);	
				}					
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_SERVOCENTERR, servocenterr);	
						servo.write(getServoCenter());
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}



void menuSetServoDirection(){


		EEPROM_readAnything(EPPROM_DIRECTION, servoDirection);		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_DIRECTION);
		lcd.setCursor(15,1);
		if(servoDirection == 0){
			lcd.print("N");
		}else{
			lcd.print("Y");
		}
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT || localKey == BTN_LEFT){
					delay(500);
					
					if(servoDirection == 0){
						servoDirection = 1;
					} else {
						servoDirection = 0;
					}
					
					lcd.setCursor(15,1);
					if(servoDirection == 0){
						lcd.print("N");
					}else{
						lcd.print("Y");
					}
				}					
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_DIRECTION, servoDirection);	
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}


void menuSetVoltage(){


		EEPROM_readAnything(EPPROM_VOLTAGEALARM, voltagealarm);		
		lcd.setCursor(0,1);
		lcd.print(STR_MENU_SETVOLTAGE);
		lcd.setCursor(10,1);
		lcd.print(voltagealarm);
		
		int thisLoop = 1;
		while(thisLoop == 1){
		
			localKey = checkKeyPad();
	
				if(localKey == BTN_RIGHT){
					delay(500);
					voltagealarm = voltagealarm + 0.1;
					if( voltagealarm > 12.00){ voltagealarm = 12.00;}	
					
					
					lcd.setCursor(10,1);
					lcd.print(voltagealarm);
					lcd.print(" ");
					if(voltagealarm < 10){
						lcd.setCursor(9,1);
						lcd.print(" ");
					}
				}	
				if(localKey == BTN_LEFT){
					delay(500);
					voltagealarm = voltagealarm - 0.1;
					if( voltagealarm < 6.00){ voltagealarm = 6.00;}	

					lcd.setCursor(10,1);
					lcd.print(voltagealarm);	
					lcd.print(" ");
					if(voltagealarm < 10){
						lcd.setCursor(9,1);
						lcd.print(" ");
					}	
				}	
				if(localKey == BTN_SEL){
						delay(500);
						EEPROM_writeAnything(EPPROM_VOLTAGEALARM, voltagealarm);	
						busyAnim(15,1);
						thisLoop = 0; //break the loop
						changeState(STATE_MENU);
				}
				
		}		
}


void menuSelectArrow(){
		lcd.setCursor(0,1); 
		lcd.print(" >");
		delay(500);
}

void switchMenuLoc(int loc){
	menuLoc = loc;
	menuRefreshLcd = 1;
	delay(500); //small delay to prevent keypad being to quick
}

			

int checkKeyPad(){
	int keyPress = keypad.getKey();
	if (keyPress != SAMPLE_WAIT){
		if(keyPress == BTN_UP){
			return BTN_UP;
		}
		if(keyPress == BTN_DOWN){
			return BTN_DOWN;
		}
		if(keyPress == BTN_LEFT){
			return BTN_LEFT;
		}	
		if(keyPress == BTN_RIGHT){
			return BTN_RIGHT;
		}	
		if(keyPress == BTN_SEL){
			return BTN_SEL;
		}		
	}
	return 0;
}

void menuWrite(char const *txt){
	if(menuRefreshLcd == 1){
		lcd.setCursor(0,1); 
		lcd.print("                ");
		lcd.setCursor(0,1); 
		lcd.print(txt);
		menuRefreshLcd = 0;
	}
}

void changeState(int x){
	state = x;
	
	if(state == STATE_MENU){
		menuLoc = MENU_RUN;
		menuRefreshLcd = 1;
	}	
	
}

void eppromInit(){


	//clear epprom if first bit not set
	if(EEPROM.read(0) != 1 || analogRead(DEFAULT_KEY_PIN) > 5){
	
		lcd.setCursor(0,0);
		lcd.print(STR_FORMAT);

		//clear the epprom
		int c = 0;
		for (int i = 0; i < 512; i++){
			EEPROM.write(i, 0);
			
			if(c == 0){
				lcd.setCursor(0,1);
				lcd.print("|");
				c = 1;
			} else {
				lcd.setCursor(0,1);
				lcd.print("-");
				c = 0;
			}
			delay(5);
			
		}
		

		EEPROM_writeAnything(EPPROM_FORMAT, 1);
		EEPROM_writeAnything(EPPROM_CHANNEL, 0);
		EEPROM_writeAnything(EPPROM_BAND, 0);
		EEPROM_writeAnything(EPPROM_SPEED, 15);
		EEPROM_writeAnything(EPPROM_KALMAN, 150);
		EEPROM_writeAnything(EPPROM_VOLTAGEALARM, 11.2);
		EEPROM_writeAnything(EPPROM_SERVOCENTERL, 1480);
		EEPROM_writeAnything(EPPROM_SERVOCENTERR, 1480);	
		EEPROM_writeAnything(EPPROM_DIRECTION, 0);
		EEPROM_writeAnything(EPPROM_CENTERHOLD, 5);
		
		EEPROM_writeAnything(EPPROM_LOWLEFT, 0);
		EEPROM_writeAnything(EPPROM_LOWRIGHT, 0);
		EEPROM_writeAnything(EPPROM_LOWMID, 0);
		EEPROM_writeAnything(EPPROM_LOWTOP, 0);
		
		EEPROM_writeAnything(EPPROM_HIGHLEFT, 0);
		EEPROM_writeAnything(EPPROM_HIGHRIGHT, 0);
		EEPROM_writeAnything(EPPROM_HIGHMID, 0);
		EEPROM_writeAnything(EPPROM_HIGHTOP, 0);			
		EEPROM_writeAnything(EPPROM_KALMANAV, 100);		


		lcd.clear();

		lcd.setCursor(0,0);
		lcd.print(STR_PRESSKEY);		
		
		int v = 0;
		int tmpKey;
		int keyLocInit = 1;
		while(v < 5){

			if(keyLocInit == 1){
				if(v == 0){
						lcd.setCursor(0,1);
						lcd.print(STR_CALIB_UP);			
				}
				if(v == 1){
						lcd.setCursor(0,1);
						lcd.print(STR_CALIB_DOWN);			
				}			
				if(v == 2){
						lcd.setCursor(0,1);
						lcd.print(STR_CALIB_LEFT);				
				}	
				if(v == 3){
						lcd.setCursor(0,1);
						lcd.print(STR_CALIB_RIGHT);			
				}	
				if(v == 4){
						lcd.setCursor(0,1);
						lcd.print(STR_CALIB_SELECT);		
				}	
				lcd.setCursor(8,1);
				lcd.print("            ");
				keyLocInit = 0;
			}
	
			if(analogRead(DEFAULT_KEY_PIN) >= 0){
				lcd.setCursor(8,1);
				

				delay(4000);
				tmpKey = analogRead(DEFAULT_KEY_PIN);

				lcd.print(STR_CALIB_OK);
				lcd.print(tmpKey);
				keyLocInit = 1;
				
				delay(500);
				
				if(v == 0){
				   UPKEY_ARV = tmpKey;
				   EEPROM_writeAnything(EPPROM_KEYUP, UPKEY_ARV);
				}
				if(v == 1){
				   DOWNKEY_ARV = tmpKey;
				   EEPROM_writeAnything(EPPROM_KEYDOWN, DOWNKEY_ARV);
				}
				if(v == 2){
				   LEFTKEY_ARV = tmpKey;
				   EEPROM_writeAnything(EPPROM_KEYLEFT, LEFTKEY_ARV);
				}
				if(v == 3){
				   RIGHTKEY_ARV = tmpKey;
				   EEPROM_writeAnything(EPPROM_KEYRIGHT, RIGHTKEY_ARV);
				}
				if(v == 4){
				   SELKEY_ARV = tmpKey;
				   EEPROM_writeAnything(EPPROM_KEYSEL, SELKEY_ARV);
				}				
				
				delay(1000);
				v++;			
			}		
		}
	}
	lcd.clear();
	
}

int ReadNumber()
{
  int rxbyte = 0;
  
  while (!((rxbyte >= '0') && (rxbyte <= '9')))
  {
    // Has some data comes in?
    if (Serial.available())
      // Read in the byte
      rxbyte = Serial.read();
  }
  
  return rxbyte - '0';
}

void SERIAL_SENDBIT1()
{
  //Serial.print("1");
  MODULE_CLK_LOW();
  delayMicroseconds(DELAY_CLOCK_US);
  
  MODULE_DATA_HIGH();
  delayMicroseconds(DELAY_CLOCK_US);
  MODULE_CLK_HIGH();
  delayMicroseconds(DELAY_CLOCK_US);
  
  MODULE_CLK_LOW();
  delayMicroseconds(DELAY_CLOCK_US);
}

void SERIAL_SENDBIT0()
{
  //Serial.print("0");
  MODULE_CLK_LOW();
  delayMicroseconds(DELAY_CLOCK_US);
  
  MODULE_DATA_LOW();
  delayMicroseconds(DELAY_CLOCK_US);
  MODULE_CLK_HIGH();
  delayMicroseconds(DELAY_CLOCK_US);
  
  MODULE_CLK_LOW();
  delayMicroseconds(DELAY_CLOCK_US);
}

void SERIAL_ENABLE_LOW(int Module)
{
  delayMicroseconds(DELAY_ENABLE_US);
  MODULE_EN_LOW(Module);
  delayMicroseconds(DELAY_ENABLE_US);
}

void SERIAL_ENABLE_HIGH(int Module)
{
  delayMicroseconds(DELAY_ENABLE_US); 
  MODULE_EN_HIGH(Module);
  delayMicroseconds(DELAY_ENABLE_US);
}

int setModuleChannelAll(int bandTmp, int channelTmp,int mode){
				setModuleChannel(PIN_MODULE1C2,  bandTmp, channelTmp);
				setModuleChannel(PIN_MODULE2C2,  bandTmp, channelTmp);
				setModuleChannel(PIN_MODULE3C2,  bandTmp, channelTmp);
				setModuleChannel(PIN_MODULE4C2,  bandTmp, channelTmp);	
				delay(200);
				int loops = 50;
				int i;
				for(i=0;i<loops;i++){
					rssiLeft = rssiLeft  + analogRead(PIN_RSSI2);
					rssiRight = rssiRight  + analogRead(PIN_RSSI0) ;
					
				}
				rssiLeft = rssiLeft/loops;
				rssiRight = rssiRight/loops;
				
				int AvgRSSI = (rssiLeft + rssiRight) / 2;


				if(rssiLeft >= 150 && rssiRight >= 150){
						if(mode == 1){
							tone(PIN_BUZZER, 3000, 500);
						}
						updateChannel();
						if(mode == 1){
							return 1;
						} else {
							return AvgRSSI;
						}
				} else {
						updateChannel();
						if(mode == 1){
							return 0;
						} else {
							return AvgRSSI;						
						}
				}
	
	
				
		
}

void setModuleChannel(int Module,int Band, int Channel)
{



  uint8_t i;
  uint16_t channelData;
  
  //channelData = pgm_read_word(&channelTable[channel]);
  channelData = channelTable[(Band*8)+Channel];
  //channelData = channelTable[Channel-1];  //subtract one as calcs start from 0
  
  //Serial.println(channelData, HEX);
  
  // bit bash out 25 bits of data
  // Order: A0-3, !R/W, D0-D19
  // A0=0, A1=0, A2=0, A3=1, RW=0, D0-19=0
  SERIAL_ENABLE_HIGH(Module);
  delay(DELAY_CLOCKEDIN_MS);
  SERIAL_ENABLE_LOW(Module);

  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT1();
  
  SERIAL_SENDBIT0();
  
  // remaining zeros
  for (i=20;i>0;i--)
    SERIAL_SENDBIT0();
  
  // Clock the data in
  SERIAL_ENABLE_HIGH(Module);
  delay(DELAY_CLOCKEDIN_MS);
  SERIAL_ENABLE_LOW(Module);

  // Second is the channel data from the lookup table
  // 20 bytes of register data are sent, but the MSB 4 bits are zeros
  // register address = 0x1, write, data0-15=channelData data15-19=0x0
  SERIAL_ENABLE_HIGH(Module);
  delay(DELAY_CLOCKEDIN_MS);
  SERIAL_ENABLE_LOW(Module);
  
  // Register 0x1
  SERIAL_SENDBIT1();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  SERIAL_SENDBIT0();
  
  // Write to register
  SERIAL_SENDBIT1();
  
  // D0-D15
  //   note: loop runs backwards as more efficent on AVR
  for (i=16;i>0;i--)
  {
    // Is bit high or low?
    if (channelData & 0x1)
    {
      SERIAL_SENDBIT1();
    }
    else
    {
      SERIAL_SENDBIT0();
    }
    
    // Shift bits along to check the next one
    channelData >>= 1;
  }
  
  // Remaining D16-D19
  for (i=4;i>0;i--)
    SERIAL_SENDBIT0();
  
  // Finished clocking data in
  SERIAL_ENABLE_HIGH(Module);
  delay(DELAY_CLOCKEDIN_MS);
  
  //MODULE_EN_HIGH(Module);
  MODULE_EN_LOW(Module);
  
  MODULE_CLK_LOW();
  MODULE_DATA_LOW();
  
  
}


void busyAnim(int x,int y){
			int loops = 10;
			int c = 0;
			int i;
			for(i=0;i<loops;i++){
				if(c == 0){
					lcd.setCursor(x,y);
					lcd.print("|");
					c = 1;
				} else {
					lcd.setCursor(x,y);
					lcd.print("-");
					c = 0;
				}
				delay(100);
			}
}

float readVoltage(){


		analogRead(PIN_VOLTAGE);
		float val =  analogRead(PIN_VOLTAGE) * 0.0117497556207234;
		
		
		if(val <= voltagealarm){
			tone(PIN_BUZZER, 3000, 500);
		}
		
		return val;
		
}

void updateVoltage(){
	float v = readVoltage();
	lcd.setCursor(10,0);
	lcd.print(v);
	lcd.print("v");
	

}


void updateChannel(){

	lcd.setCursor(0,0);
	lcd.print("CH");
	

	lcd.print(band + 1);
	lcd.print(channel + 1);		

    uint16_t freq = channelTableFreq[(band*8)+channel];

	lcd.print(" ");
	lcd.print(freq);


}

float kalman_update(float measurement)
{
  //static int lcnt=0;
  static float x=rssiDiff; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain

  // update the prediction value
  p = p + kalman_q;

  // update based on measurement
  k = p / (p + kalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

  return x;
}


float div_kalman_update1(float measurement)
{
  //static int lcnt=0;
  static float x=rssiMidRaw; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain

  // update the prediction value
  p = p + div1_kalman_q;

  // update based on measurement
  k = p / (p + div1_kalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

  return x;
}

float div_kalman_update2(float measurement)
{
  //static int lcnt=0;
  static float x=rssiTopRaw; //value
  static float p=100; //estimation error covariance
  static float k=0; //kalman gain

  // update the prediction value
  p = p + div2_kalman_q;

  // update based on measurement
  k = p / (p + div2_kalman_r);
  x = x + k * (measurement - x);
  p = (1 - k) * p;

  return x;
}


