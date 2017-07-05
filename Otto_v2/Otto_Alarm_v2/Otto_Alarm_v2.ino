
//----------------------------------------------------------------
//-- Niki Alarm  v2
//
// added RGB LED, enabled interrupts while serial active otherwise after BT disconnect
// interrupts remain blocked, serial set at 57600. - by Lev
//
//-----------------------------------------------------------------

#include <Servo.h>
#include <Oscillator.h>
#include <EEPROM.h>
//#include <BatReader.h> //uncomment this if you're using step up converter
#include <US.h>
#include <EnableInterrupt.h>
#include <OttoSerialCommand.h>
#include <Otto.h>
Otto Otto;  //This is Otto!!
OttoSerialCommand SCmd;  //The SerialCommand object

//--Configuring LEDMatrix display
MaxMatrix ledmatrix = MaxMatrix(10, 11, 12, 1); // DIN=10 CS=11 CLK=12

//---------------------------------------------------------
//-- Configuration of pins where the servos are attached
/*
          ---------------
         |               |
         |     O   O     |
         |               |
  YR ==> |               | <== YL
          ---------------
             ||     ||
             ||     ||
             ||     ||
  RR ==>   -----   ------  <== RL
           -----   ------
*/

#define PIN_YL 2 //servo[0]
#define PIN_YR 3 //servo[1]
#define PIN_RL 4 //servo[2]
#define PIN_RR 5 //servo[3]
//---------------------------------------------------------

//---Otto Buttons
#define PIN_SecondButton 6
#define PIN_ThirdButton 7


///////////////////////////////////////////////////////////////////
//-- Global Variables -------------------------------------------//
///////////////////////////////////////////////////////////////////
#define BAT_MAX  4.2
#define BAT_MIN 3.2
#define SLOPE 100/(BAT_MAX - BAT_MIN)
#define OFFSET  (100*BAT_MIN)/(BAT_MAX - BAT_MIN)

const char programID[] = "ZOWI_Alarm_v2"; //Each program will have a ID

//-- Movement parameters
int T = 1000;            //Initial duration of movement
int moveId = 0;          //Number of movement
int moveSize = 15;       //Asociated with the height of some movements


//---------------------------------------------------------
//-- Adivinawi has 5 modes:
//--    * MODE = 0: Otto is awaiting
//--    * MODE = 1: Arming alarm system
//--    * MODE = 2: Otto Guardian
//--    * MODE = 3: --
//--    * MODE = 4: OttoPAD or any Teleoperation mode (listening SerialPort).
//--
volatile int MODE = 0; //State of Otto in the principal state machine.
volatile int currentMode = 1; //To remember current mode when changing modes
//---------------------------------------------------------


volatile bool buttonPushed = false; //Variable to remember when a button has been pushed
volatile bool buttonAPushed = false; //Variable to remember when A button has been pushed
volatile bool buttonBPushed = false; //Variable to remember when B button has been pushed

unsigned long previousMillis = 0;

int randomDance = 0;
int randomSteps = 0;

bool obstacleDetected = false;

bool alarmActivated = false;
int initDistance = 999;
unsigned long int arming_symbol =   0b00111111100001100001100001111111;
unsigned long int alarm_symbol =    0b00111111111111111111111111111111;

int angryPos2[4] =    {90, 90, 70, 110};
int headLeft2[4] =    {110, 110, 90, 90};
int headRight2[4] =   {70, 70, 90, 90};

//Function that reads voltage of battery
long readVcc() {
  // Read 1.1V reference against AVcc
  // set the reference to Vcc and the measurement to the internal 1.1V reference
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both

  long result = (high << 8) | low;

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  return result; // Vcc in millivolts
}



///////////////////////////////////////////////////////////////////
//-- Setup ------------------------------------------------------//
///////////////////////////////////////////////////////////////////
void setup() {

  //Serial communication initialization
  Serial.begin(57600);

  pinMode(PIN_SecondButton, INPUT);
  pinMode(PIN_ThirdButton, INPUT);

  //Set the servo pins
  Otto.init(PIN_YL, PIN_YR, PIN_RL, PIN_RR, true);

  //Uncomment this to set the servo trims manually and save on EEPROM
  //Otto.setTrims(TRIM_YL, TRIM_YR, TRIM_RL, TRIM_RR);
  //Otto.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.

  //Set a random seed
  randomSeed(analogRead(A6));

  //Interrumptions
  enableInterrupt(PIN_SecondButton, secondButtonPushed, RISING);
  enableInterrupt(PIN_ThirdButton, thirdButtonPushed, RISING);

  //Setup callbacks for SerialCommand commands
  SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
  SCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck
  SCmd.addCommand("E", requestName);
  SCmd.addCommand("B", requestBattery);
  SCmd.addCommand("I", requestProgramId);
  SCmd.addDefaultHandler(receiveStop);

  //Starting up LED matrix display
  ledmatrix.init();
  ledmatrix.setIntensity(10);

  //Otto wake up!
  //digitalWrite(ledR, random(2));
  //digitalWrite(ledG, random(2));
  //digitalWrite(ledB, random(2));
  Otto.sing(S_connection);
  delay(500);
  //digitalWrite(ledR, random(2));
  //digitalWrite(ledG, random(2));
  //digitalWrite(ledB, random(2));
  Otto.home();


  //Send Otto name, programID & battery level.
  requestName();
  delay(50);
  requestProgramId();
  delay(50);
  requestBattery();

  //Checking battery
  OttoLowBatteryAlarm();


  // Animation Uuuuuh - A little moment of initial surprise
  //-----
  for (int i = 0; i < 2; i++) {
    for (int i = 0; i < 8; i++) {
      if (buttonPushed) {
        break;
      }
      Otto.putAnimationMouth(littleUuh, i);
      delay(150);
    }
  }
  //-----


  //Smile for a happy Otto :)
  if (!buttonPushed) {

    Otto.playGesture(OttoHappy);
    Otto.sing(S_happy);
    delay(200);
  }

  if (!buttonPushed) {
    Otto.putMouth(happyOpen);
  }

  previousMillis = millis();
}



///////////////////////////////////////////////////////////////////
//-- Principal Loop ---------------------------------------------//
///////////////////////////////////////////////////////////////////
void loop() {

  if (Serial.available() > 0 && MODE != 4) {

    MODE = 4;
    Otto.putMouth(happyOpen);
    /*
        //Disable Pin Interruptions
        disableInterrupt(PIN_SecondButton);
        disableInterrupt(PIN_ThirdButton);

        buttonPushed = false;
    */
  }


  //First attemp to initial software
  if (buttonPushed) {

    Otto.home();
    //digitalWrite(ledR, random(2));
    //digitalWrite(ledG, random(2));
    //digitalWrite(ledB, random(2));
    delay(100); //Wait for all buttons
    Otto.sing(S_buttonPushed);
    delay(200); //Wait for all buttons

    if      ( buttonAPushed && !buttonBPushed) {
      MODE = 1;
      Otto.sing(S_mode1);
    }
    else if (!buttonAPushed && buttonBPushed) {
      MODE = 2;
      Otto.sing(S_mode2);
    }
    else if ( buttonAPushed && buttonBPushed) {
      MODE = 2;  //else
      Otto.sing(S_mode2);
    }

    Otto.putMouth(MODE);

    int showTime = 1500;
    while ((showTime > 0)) { //Wait to show the MODE number

      showTime -= 10;
      delay(10);
    }

    Otto.clearMouth();


    buttonPushed = false;
    buttonAPushed = false;
    buttonBPushed = false;
    alarmActivated = false;
    previousMillis = millis();

  } else {

    switch (MODE) {

      //-- MODE 0 - Otto is awaiting
      //---------------------------------------------------------
      case 0:

        //Every 80 seconds in this mode, Otto falls asleep
        if (millis() - previousMillis >= 80000) {
          //digitalWrite(ledR, HIGH);
          //digitalWrite(ledG, LOW);
          //digitalWrite(ledB, LOW);
          OttoSleeping_withInterrupts(); //ZZzzzzz...
          //digitalWrite(ledR, random(2));
          //digitalWrite(ledG, random(2));
          //digitalWrite(ledB, random(2));
          previousMillis = millis();
        }

        break;


      //-- MODE 1 - Otto Arming alarm system
      //---------------------------------------------------------
      case 1:


        //Arming alarm system
        if (alarmActivated == false) {

          OttoArmingAlarmSystem();

        } else {
          //digitalWrite(ledR, LOW);
          //digitalWrite(ledG, LOW);
          //digitalWrite(ledB, HIGH);
          delay(100);
          int obstacleDistance = Otto.getDistance();
          int noise = Otto.getNoise();
          delay(100);

          //ALARM!!!!
          if ((noise >= 600) || (obstacleDistance < initDistance)) {

            delay(50);
            while (!buttonPushed) {
              //digitalWrite(ledR, HIGH);
              //digitalWrite(ledG, LOW);
              //digitalWrite(ledB, LOW);
              Otto.putMouth(alarm_symbol, 0);
              Otto.bendTones (note_A5, note_A7, 1.04, 5, 2);  //A5 = 880 , A7 = 3520

              delay(20);
              //digitalWrite(ledR, LOW);
              //digitalWrite(ledG, LOW);
              //digitalWrite(ledB, LOW);
              Otto.bendTones (note_A7, note_A5, 1.02, 5, 2);  //A5 = 880 , A7 = 3520
              Otto.clearMouth();
              delay(100);
            }
          }

        }




        break;


      //-- MODE 2 - Otto Guardian
      //---------------------------------------------------------
      case 2:

        //Arming alarm system
        if (alarmActivated == false) {

          OttoArmingAlarmSystem();

        } else {
          //digitalWrite(ledR, HIGH);
          //digitalWrite(ledG, LOW);
          //digitalWrite(ledB, HIGH);
          delay(100);
          int obstacleDistance = Otto.getDistance();
          int noise = Otto.getNoise();
          delay(100);

          //ALARM!!!!
          if ((noise >= 600) || (obstacleDistance < initDistance)) {

            delay(50);
            while (!buttonPushed) {
              //digitalWrite(ledR, HIGH);
              //digitalWrite(ledG, LOW);
              //digitalWrite(ledB, LOW);
              Otto.putMouth(alarm_symbol, 0);
              Otto.bendTones (note_A5, note_A7, 1.04, 5, 2);  //A5 = 880 , A7 = 3520
              delay(20);
              //digitalWrite(ledR, LOW);
              //digitalWrite(ledG, LOW);
              //digitalWrite(ledB, LOW);
              Otto.bendTones (note_A7, note_A5, 1.02, 5, 2);  //A5 = 880 , A7 = 3520
              Otto.clearMouth();
              delay(100);
            }
          }

          if (millis() - previousMillis >= 8000) { //8sec
            //digitalWrite(ledR, random(2));
            //digitalWrite(ledG, random(2));
            //digitalWrite(ledB, random(2));
            OttoGuardian();

            if (!buttonPushed) {

              delay(100);
              initDistance = Otto.getDistance();
              delay(100);
              initDistance -= 10;
            }

            previousMillis = millis();

          }

        }

        break;


      case 4:

        SCmd.readSerial();
        break;


      default:
        MODE = 4;
        break;
    }

  }

}



///////////////////////////////////////////////////////////////////
//-- Functions --------------------------------------------------//
///////////////////////////////////////////////////////////////////

//-- Function executed when second button is pushed
void secondButtonPushed() {

  buttonAPushed = true;

  if (!buttonPushed) {
    buttonPushed = true;
    Otto.putMouth(smallSurprise);
  }
}

//-- Function executed when third button is pushed
void thirdButtonPushed() {

  buttonBPushed = true;

  if (!buttonPushed) {
    buttonPushed = true;
    Otto.putMouth(smallSurprise);
  }
}



//-- Function to receive Stop command.
void receiveStop() {

  sendAck();
  Otto.home();
  sendFinalAck();

}


//-- Function to receive Name command
void receiveName() {

  //sendAck & stop if necessary
  sendAck();
  Otto.home();

  char newOttoName[11] = "";  //Variable to store data read from Serial.
  int eeAddress = 5;          //Location we want the data to be in EEPROM.
  char *arg;
  arg = SCmd.next();

  if (arg != NULL) {

    //Complete newOttoName char string
    int k = 0;
    while ((*arg) && (k < 11)) {
      newOttoName[k] = *arg++;
      k++;
    }

    EEPROM.put(eeAddress, newOttoName);
  }
  else
  {
    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();
  }

  sendFinalAck();

}


//-- Function to send Otto's name
void requestName() {

  Otto.home(); //stop if necessary

  char actualOttoName[11] = ""; //Variable to store data read from EEPROM.
  int eeAddress = 5;            //EEPROM address to start reading from

  //Get the float data from the EEPROM at position 'eeAddress'
  EEPROM.get(eeAddress, actualOttoName);

  Serial.print(F("&&"));
  Serial.print(F("E "));
  Serial.print(actualOttoName);
  Serial.println(F("%%"));
  Serial.flush();
}



//-- Function to send battery voltage percent
void requestBattery() {

  Otto.home();  //stop if necessary

  int batteryLevel = ((readVcc() / 1000.00) * SLOPE) - OFFSET;
  
  //double batteryLevel = Otto.getBatteryLevel(); //uncomment this one and comment out above one when using step up converter
  
  if (batteryLevel > 100) {
    batteryLevel = 100;
  };
  Serial.print(F("&&"));
  Serial.print(F("B "));
  Serial.print(batteryLevel);
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send program ID
void requestProgramId() {

  Otto.home();   //stop if necessary

  Serial.print(F("&&"));
  Serial.print(F("I "));
  Serial.print(programID);
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send Ack comand (A)
void sendAck() {

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("A"));
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send final Ack comand (F)
void sendFinalAck() {

  delay(30);

  Serial.print(F("&&"));
  Serial.print(F("F"));
  Serial.println(F("%%"));
  Serial.flush();
}



//-- Functions with animatics
//--------------------------------------------------------

void OttoLowBatteryAlarm() {
  
  int batteryLevel = ((readVcc() / 1000.00) * SLOPE) - OFFSET;
  
  //double batteryLevel = Otto.getBatteryLevel(); //uncomment this one and comment out above one when using step up converter
  
  if (batteryLevel < 30) {
    while (!buttonPushed) {

      //digitalWrite(ledR, LOW);
      //digitalWrite(ledG, LOW);
      //digitalWrite(ledB, LOW);
      Otto.putMouth(thunder);
      Otto.bendTones (880, 2000, 1.04, 8, 3);  //A5 = 880

      delay(30);

      Otto.bendTones (2000, 880, 1.02, 8, 3);  //A5 = 880
      Otto.clearMouth();
      //digitalWrite(ledR, HIGH);
      //digitalWrite(ledG, LOW);
      //digitalWrite(ledB, LOW);
      delay(500);
    }
  }
}

void OttoSleeping_withInterrupts() {

  int bedPos_0[4] = {100, 80, 60, 120}; //{100, 80, 40, 140}

  if (!buttonPushed) {
    Otto._moveServos(700, bedPos_0);  //800
  }

  for (int i = 0; i < 4; i++) {

    if (buttonPushed) {
      break;
    }
    digitalWrite(ledR, HIGH);
    digitalWrite(ledG, HIGH);
    digitalWrite(ledB, HIGH);
    Otto.putAnimationMouth(dreamMouth, 0);
    Otto.bendTones (100, 200, 1.04, 10, 10);

    if (buttonPushed) {
      break;
    }
    Otto.putAnimationMouth(dreamMouth, 1);
    Otto.bendTones (200, 300, 1.04, 10, 10);

    if (buttonPushed) {
      break;
    }
    Otto.putAnimationMouth(dreamMouth, 2);
    Otto.bendTones (300, 500, 1.04, 10, 10);

    delay(500);
    digitalWrite(ledR, LOW);
    digitalWrite(ledG, LOW);
    digitalWrite(ledB, LOW);
    if (buttonPushed) {
      break;
    }
    Otto.putAnimationMouth(dreamMouth, 1);
    Otto.bendTones (400, 250, 1.04, 10, 1);

    if (buttonPushed) {
      break;
    }
    Otto.putAnimationMouth(dreamMouth, 0);
    Otto.bendTones (250, 100, 1.04, 10, 1);

    delay(500);
  }

  if (!buttonPushed) {
    Otto.putMouth(lineMouth);
    Otto.sing(S_cuddly);
  }

  Otto.home();
  if (!buttonPushed) {
    Otto.putMouth(happyOpen);
  }
}

void OttoArmingAlarmSystem() {

  int countDown = 10000; //10 sec
  while ((countDown > 0) && (!buttonPushed)) {
    digitalWrite(ledR, LOW);
    digitalWrite(ledG, HIGH);
    digitalWrite(ledB, LOW);
    countDown -= 1000;
    Otto.putMouth(arming_symbol, 0);
    Otto._tone(note_A7, 50, 0); //bip'
    Otto.clearMouth();
    delay(300);
    digitalWrite(ledR, LOW);
    digitalWrite(ledG, LOW);
    digitalWrite(ledB, LOW);
    delay(650);
    if (buttonPushed) {
      break;
    }

  }

  if (!buttonPushed) {
    alarmActivated = true;
    delay(100);
    initDistance = Otto.getDistance();
    delay(100);
    initDistance -= 10;
    previousMillis = millis();
  }
}

void OttoGuardian() {

  int fretfulPos[4] =  {90, 90, 90, 110};

  if (!buttonPushed) {
    Otto.putMouth(smallSurprise);
    delay(100);
    Otto.sing(S_cuddly);
    delay(500);
  }

  if (!buttonPushed) {
    Otto.putMouth(angry);
    Otto.bendTones(note_A5, note_D6, 1.02, 20, 4);
    Otto.bendTones(note_A5, note_E5, 1.02, 20, 4);
    delay(300);
    Otto.putMouth(lineMouth);
  }


  for (int i = 0; i < 4; i++) {
    if (buttonPushed) {
      break;
    }
    Otto._moveServos(100, fretfulPos);
    Otto.home();
  }

  if (!buttonPushed) {
    Otto.putMouth(angry);
    delay(500);
  }


  if (!buttonPushed) {
    Otto._moveServos(1000, headLeft2);
    delay(400);
  }

  if (!buttonPushed) {
    Otto.home();
    delay(400);
  }

  if (!buttonPushed) {
    Otto._moveServos(1000, headRight2);
    delay(400);
  }

  if (!buttonPushed) {
    delay(300);
    Otto.home();
    Otto.putMouth(smile);
    delay(100);
  }

  if (!buttonPushed) {
    Otto.sing(S_happy_short);
    delay(800);
    Otto.clearMouth();
  }

}
