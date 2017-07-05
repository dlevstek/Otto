//----------------------------------------------------------------
//-- Zowi basic firmware v2 adapted to Otto
//-- (c) BQ. Released under a GPL licencse
//-- 04 December 2015
//-- Authors:  Anita de Prado: ana.deprado@bq.com
//--           Jose Alberca:   jose.alberca@bq.com
//--           Javier Isabel:  javier.isabel@bq.com
//--           Juan Gonzalez (obijuan): juan.gonzalez@bq.com
//--           Irene Sanz : irene.sanz@bq.com
//-----------------------------------------------------------------
//-- Experiment with all the features that Otto has thanks to Otto!
//-----------------------------------------------------------------

//-- Otto basic firmware v2
//
//Modified 8x8 matrix library (MaxMatrix) to use less memory (Buffer set to 35 instead 80)
//Added RGB led as nose
//Using 2 regular buttons
//Modified Otto.cpp and Otto.h for RGB led and to disable default voltage reading
//Modified OttoSerialCommand.h to use less memory
//Serial set to 57600 - speed used to program Arduino Nano so it can be programmed over BT
//Enabled pin interrupts on serial connection because else we can't exit serial mode without reset
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Modified voltage reading to use internal voltage reading - works only if not using step up converter.        //
//If using step up you have to connect additional + from battery to A7 pin and uncomment #include <BatReader.h>//
//and change reading in requestBattery(); and OttoLowBatteryAlarm();                                           //
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//
// - by Davor Levstek
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

MaxMatrix ledmatrix = MaxMatrix(10, 11, 12, 1); // DIN=10 CS=11 CLK=12

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

//Kept the name so android app doesen't give warnings
const char programID[] = "ZOWI_BASE_v2"; //Each program will have a ID

const char name_fac = '$'; //Factory name
const char name_fir = '#'; //First name

//-- Movement parameters
int T = 1000;            //Initial duration of movement
int moveId = 0;          //Number of movement
int moveSize = 15;       //Asociated with the height of some movements

//---------------------------------------------------------
//-- Otto has 5 modes:
//--    * MODE = 0: Otto is awaiting
//--    * MODE = 1: Dancing mode!
//--    * MODE = 2: Obstacle detector mode
//--    * MODE = 3: Noise detector mode
//--    * MODE = 4: OttoPAD or any Teleoperation mode (listening SerialPort).
//--
//---------------------------------------------------------
volatile int MODE = 0; //State of Otto in the principal state machine.
volatile bool buttonPushed = false; //Variable to remember when a button has been pushed
volatile bool buttonAPushed = false; //Variable to remember when A button has been pushed
volatile bool buttonBPushed = false; //Variable to remember when B button has been pushed

unsigned long previousMillis = 0;

int randomDance = 0;
int randomSteps = 0;

bool obstacleDetected = false;

//Function that reads internal voltage of Arduino
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

  //Starting up LED matrix display
  ledmatrix.init();
  ledmatrix.setIntensity(10);

  //Uncomment this to set the servo trims manually and save on EEPROM
  //Otto.setTrims(TRIM_YL, TRIM_YR, TRIM_RL, TRIM_RR);
  //Otto.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.

  //Set a random seed for RGB led
  randomSeed(analogRead(A6));

  //Interrumptions
  enableInterrupt(PIN_SecondButton, secondButtonPushed, RISING);
  enableInterrupt(PIN_ThirdButton, thirdButtonPushed, RISING);

  //Setup callbacks for SerialCommand commands
  SCmd.addCommand("S", receiveStop);      //  sendAck & sendFinalAck
  SCmd.addCommand("L", receiveLED);       //  sendAck & sendFinalAck
  SCmd.addCommand("T", recieveBuzzer);    //  sendAck & sendFinalAck
  SCmd.addCommand("M", receiveMovement);  //  sendAck & sendFinalAck
  SCmd.addCommand("H", receiveGesture);   //  sendAck & sendFinalAck
  SCmd.addCommand("K", receiveSing);      //  sendAck & sendFinalAck
  SCmd.addCommand("C", receiveTrims);     //  sendAck & sendFinalAck
  SCmd.addCommand("G", receiveServo);     //  sendAck & sendFinalAck
  SCmd.addCommand("R", receiveName);      //  sendAck & sendFinalAck
  SCmd.addCommand("E", requestName);
  SCmd.addCommand("D", requestDistance);
  SCmd.addCommand("N", requestNoise);
  SCmd.addCommand("B", requestBattery);
  SCmd.addCommand("I", requestProgramId);
  SCmd.addDefaultHandler(receiveStop);

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


  //If Otto's name is '&' (factory name) means that is the first time this program is executed.
  //This first time, Otto mustn't do anything. Just born at the factory!
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5) == name_fac) {

    EEPROM.put(5, name_fir); //From now, the name is '#'
    EEPROM.put(6, '\0');
    Otto.putMouth(culito);

    while (true) {
      delay(1000);
    }
  }

  //Send Otto name, programID & battery level.
  requestName();
  delay(50);
  requestProgramId();
  delay(50);
  requestBattery();

  OttoLowBatteryAlarm();


  // Animation Uuuuuh - A little moment of initial surprise
  for (int i = 0; i < 2; i++) {
    for (int i = 0; i < 8; i++) {
      if (buttonPushed) {
        break;
      }
      Otto.putAnimationMouth(littleUuh, i);
      delay(150);
    }
  }


  //Smile for a happy Otto :)
  if (!buttonPushed) {
    Otto.putMouth(smile);
    Otto.sing(S_happy);
    delay(200);
  }


  //If Otto's name is '#' means that Otto hasn't been baptized
  //In this case, Otto does a longer greeting
  //5 = EEPROM address that contains first name character
  if (EEPROM.read(5) == name_fir) {

    if (!buttonPushed) {
      Otto.jump(1, 700);
      delay(200);
    }

    if (!buttonPushed) {
      Otto.shakeLeg(1, T, 1);
    }

    if (!buttonPushed) {
      Otto.putMouth(smallSurprise);
      Otto.swing(2, 800, 20);
      Otto.home();
    }
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
         buttonPushed=false;
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
      MODE = 3;  //else
      Otto.sing(S_mode3);
    }
    Otto.putMouth(MODE);
    int showTime = 1500;
    while ((showTime > 0)) { //Wait to show the MODE number

      showTime -= 10;
      delay(10);
    }

    Otto.putMouth(happyOpen);

    buttonPushed = false;
    buttonAPushed = false;
    buttonBPushed = false;

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


      //-- MODE 1 - Dance Mode!
      //---------------------------------------------------------
      case 1:

        randomDance = random(5, 21); //5,20
        if ((randomDance > 14) && (randomDance < 19)) {
          randomSteps = 1;
          T = 1600;
        }
        else {
          randomSteps = random(3, 6); //3,5
          T = 1000;
        }

        Otto.putMouth(random(10, 21));
        //digitalWrite(ledR, random(2));
        //digitalWrite(ledG, random(2));
        //digitalWrite(ledB, random(2));
        for (int i = 0; i < randomSteps; i++) {
          move(randomDance);
          if (buttonPushed) {
            break;
          }
        }
        break;


      //-- MODE 2 - Obstacle detector mode
      //---------------------------------------------------------
      case 2:
        if (obstacleDetected) {
          //digitalWrite(ledR, HIGH);
          //digitalWrite(ledG, LOW);
          //digitalWrite(ledB, LOW);
          if (!buttonPushed) {
            Otto.putMouth(bigSurprise);
            Otto.sing(S_surprise);
            Otto.jump(5, 500);
          }

          if (!buttonPushed) {
            Otto.putMouth(confused);
            Otto.sing(S_cuddly);
          }

          //Otto takes two steps back
          for (int i = 0; i < 3; i++) {
            if (buttonPushed) {
              break;
            }
            //digitalWrite(ledR, random(2));
            //digitalWrite(ledG, random(2));
            //digitalWrite(ledB, random(2));
            Otto.walk(1, 1300, -1);
          }

          delay(100);
          obstacleDetector();
          delay(100);


          //If there are no obstacles and no button is pressed, Otto shows a smile
          if ((obstacleDetected == true) || (buttonPushed == true)) {
            break;
          }
          else {
            Otto.putMouth(smile);
            delay(50);
            //digitalWrite(ledR, random(2));
            //digitalWrite(ledG, random(2));
            //digitalWrite(ledB, random(2));
            obstacleDetector();
          }


          //If there are no obstacles and no button is pressed, Otto shows turns left
          for (int i = 0; i < 3; i++) {
            if ((obstacleDetected == true) || (buttonPushed == true)) {
              break;
            }
            else {
              //digitalWrite(ledR, random(2));
              //digitalWrite(ledG, random(2));
              //digitalWrite(ledB, random(2));
              Otto.turn(1, 1000, 1);
              obstacleDetector();
            }
          }

          //If there are no obstacles and no button is pressed, Otto is happy
          if ((obstacleDetected == true) || (buttonPushed == true)) {
            break;
          }
          else {
            Otto.home();
            //digitalWrite(ledR, random(2));
            //digitalWrite(ledG, random(2));
            //digitalWrite(ledB, random(2));
            Otto.putMouth(happyOpen);
            Otto.sing(S_happy_short);
            delay(200);
          }


        } else {

          Otto.walk(1, 1000, 1); //Otto walk straight
          obstacleDetector();
        }

        break;


      //-- MODE 3 - Noise detector mode
      //---------------------------------------------------------
      case 3:

        if (Otto.getNoise() >= 600) { //740
          delay(50);  //Wait for the possible 'lag' of the button interruptions.
          //Sometimes, the noise sensor detect the button before the interruption takes efect
          //digitalWrite(ledR, HIGH);
          //digitalWrite(ledG, LOW);
          //digitalWrite(ledB, LOW);
          if (!buttonPushed) {

            Otto.putMouth(bigSurprise);
            Otto.sing(S_OhOoh);

            if (buttonPushed) {
              break;
            }
            Otto.putMouth(random(10, 21));
            randomDance = random(5, 21);
            move(randomDance);
            Otto.home();
            //digitalWrite(ledR, random(2));
            //digitalWrite(ledG, random(2));
            //digitalWrite(ledB, random(2));
            delay(200); //Wait for possible noise of the servos while get home
          }

          if (!buttonPushed) {
            Otto.putMouth(happyOpen);

          }
        }
        break;


      //-- MODE 4 - OttoPAD or any Teleoperation mode (listening SerialPort)
      //---------------------------------------------------------
      case 4:

        SCmd.readSerial();

        //If Otto is moving yet
        if (Otto.getRestState() == false) {
          move(moveId);
        }

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


//-- Function to read distance sensor & to actualize obstacleDetected variable
void obstacleDetector() {

  int distance = Otto.getDistance();

  if (distance < 15) {
    obstacleDetected = true;
  } else {
    obstacleDetected = false;
  }
}


//-- Function to receive Stop command.
void receiveStop() {

  sendAck();
  Otto.home();
  sendFinalAck();

}


//-- Function to receive LED commands
void receiveLED() {

  //sendAck & stop if necessary
  sendAck();
  Otto.home();

  //Examples of receiveLED Bluetooth commands
  unsigned long int matrix;
  char *arg;
  char *endstr;
  arg = SCmd.next();
  //Serial.println (arg);
  if (arg != NULL) {
    matrix = strtoul(arg, &endstr, 2); // Converts a char string to unsigned long integer
    Otto.putMouth(matrix, false);
  } else {
    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();
  }
  sendFinalAck();
}


//-- Function to receive buzzer commands
void recieveBuzzer() {
  sendAck();
  Otto.home();
  bool error = false;
  int frec;
  int duration;
  char *arg;

  arg = SCmd.next();
  if (arg != NULL) {
    frec = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    duration = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  if (error == true) {

    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();

  } else {

    Otto._tone(frec, duration, 1);
  }

  sendFinalAck();

}


//-- Function to receive TRims commands
void receiveTrims() {

  //sendAck & stop if necessary
  sendAck();
  Otto.home();

  int trim_YL, trim_YR, trim_RL, trim_RR;

  //Definition of Servo Bluetooth command
  //C trim_YL trim_YR trim_RL trim_RR
  //Examples of receiveTrims Bluetooth commands
  //C 20 0 -8 3
  bool error = false;
  char *arg;
  arg = SCmd.next();
  if (arg != NULL) {
    trim_YL = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    trim_YR = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    trim_RL = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    trim_RR = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  if (error == true) {

    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();

  } else { //Save it on EEPROM
    Otto.setTrims(trim_YL, trim_YR, trim_RL, trim_RR);
    Otto.saveTrimsOnEEPROM(); //Uncomment this only for one upload when you finaly set the trims.
  }

  sendFinalAck();

}


//-- Function to receive Servo commands
void receiveServo() {

  sendAck();
  moveId = 30;

  //Definition of Servo Bluetooth command
  //G  servo_YL servo_YR servo_RL servo_RR
  //Example of receiveServo Bluetooth commands
  //G 90 85 96 78
  bool error = false;
  char *arg;
  int servo_YL, servo_YR, servo_RL, servo_RR;

  arg = SCmd.next();
  if (arg != NULL) {
    servo_YL = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    servo_YR = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    servo_RL = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    servo_RR = atoi(arg);  // Converts a char string to an integer
  }
  else {
    error = true;
  }

  if (error == true) {

    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();

  } else { //Update Servo:

    int servoPos[4] = {servo_YL, servo_YR, servo_RL, servo_RR};
    Otto._moveServos(200, servoPos);   //Move 200ms

  }

  sendFinalAck();

}


//-- Function to receive movement commands
void receiveMovement() {

  sendAck();

  if (Otto.getRestState() == true) {
    Otto.setRestState(false);
  }

  //Definition of Movement Bluetooth commands
  //M  MoveID  T   MoveSize
  char *arg;
  arg = SCmd.next();
  if (arg != NULL) {
    moveId = atoi(arg);
  }
  else {
    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();
    moveId = 0; //stop
  }

  arg = SCmd.next();
  if (arg != NULL) {
    T = atoi(arg);
  }
  else {
    T = 1000;
  }

  arg = SCmd.next();
  if (arg != NULL) {
    moveSize = atoi(arg);
  }
  else {
    moveSize = 15;
  }
}


//-- Function to execute the right movement according the movement command received.
void move(int moveId) {

  bool manualMode = false;

  switch (moveId) {
    case 0:
      Otto.home();
      break;
    case 1: //M 1 1000
      Otto.walk(1, T, 1);
      break;
    case 2: //M 2 1000
      Otto.walk(1, T, -1);
      break;
    case 3: //M 3 1000
      Otto.turn(1, T, 1);
      break;
    case 4: //M 4 1000
      Otto.turn(1, T, -1);
      break;
    case 5: //M 5 1000 30
      Otto.updown(1, T, moveSize);
      break;
    case 6: //M 6 1000 30
      Otto.moonwalker(1, T, moveSize, 1);
      break;
    case 7: //M 7 1000 30
      Otto.moonwalker(1, T, moveSize, -1);
      break;
    case 8: //M 8 1000 30
      Otto.swing(1, T, moveSize);
      break;
    case 9: //M 9 1000 30
      Otto.crusaito(1, T, moveSize, 1);
      break;
    case 10: //M 10 1000 30
      Otto.crusaito(1, T, moveSize, -1);
      break;
    case 11: //M 11 1000
      Otto.jump(1, T);
      break;
    case 12: //M 12 1000 30
      Otto.flapping(1, T, moveSize, 1);
      break;
    case 13: //M 13 1000 30
      Otto.flapping(1, T, moveSize, -1);
      break;
    case 14: //M 14 1000 20
      Otto.tiptoeSwing(1, T, moveSize);
      break;
    case 15: //M 15 500
      Otto.bend(1, T, 1);
      break;
    case 16: //M 16 500
      Otto.bend(1, T, -1);
      break;
    case 17: //M 17 500
      Otto.shakeLeg(1, T, 1);
      break;
    case 18: //M 18 500
      Otto.shakeLeg(1, T, -1);
      break;
    case 19: //M 19 500 20
      Otto.jitter(1, T, moveSize);
      break;
    case 20: //M 20 500 15
      Otto.ascendingTurn(1, T, moveSize);
      break;
    default:
      manualMode = true;
      break;
  }

  if (!manualMode) {
    sendFinalAck();
  }

}


//-- Function to receive gesture commands
void receiveGesture() {

  //sendAck & stop if necessary
  sendAck();
  Otto.home();

  //Definition of Gesture Bluetooth commands
  //H  GestureID
  int gesture = 0;
  char *arg;
  arg = SCmd.next();
  if (arg != NULL) {
    gesture = atoi(arg);
  }
  else
  {
    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();
  }

  switch (gesture) {
    case 1: //H 1
      Otto.playGesture(OttoHappy);
      break;
    case 2: //H 2
      Otto.playGesture(OttoSuperHappy);
      break;
    case 3: //H 3
      Otto.playGesture(OttoSad);
      break;
    case 4: //H 4
      Otto.playGesture(OttoSleeping);
      break;
    case 5: //H 5
      Otto.playGesture(OttoFart);
      break;
    case 6: //H 6
      Otto.playGesture(OttoConfused);
      break;
    case 7: //H 7
      Otto.playGesture(OttoLove);
      break;
    case 8: //H 8
      Otto.playGesture(OttoAngry);
      break;
    case 9: //H 9
      Otto.playGesture(OttoFretful);
      break;
    case 10: //H 10
      Otto.playGesture(OttoMagic);
      break;
    case 11: //H 11
      Otto.playGesture(OttoWave);
      break;
    case 12: //H 12
      Otto.playGesture(OttoVictory);
      break;
    case 13: //H 13
      Otto.playGesture(OttoFail);
      break;
    default:
      break;
  }

  sendFinalAck();
}

//-- Function to receive sing commands
void receiveSing() {

  //sendAck & stop if necessary
  sendAck();
  Otto.home();

  //Definition of Sing Bluetooth commands
  //K  SingID
  int sing = 0;
  char *arg;
  arg = SCmd.next();
  if (arg != NULL) {
    sing = atoi(arg);
  }
  else
  {
    Otto.putMouth(xMouth);
    delay(2000);
    Otto.clearMouth();
  }

  switch (sing) {
    case 1: //K 1
      Otto.sing(S_connection);
      break;
    case 2: //K 2
      Otto.sing(S_disconnection);
      break;
    case 3: //K 3
      Otto.sing(S_surprise);
      break;
    case 4: //K 4
      Otto.sing(S_OhOoh);
      break;
    case 5: //K 5
      Otto.sing(S_OhOoh2);
      break;
    case 6: //K 6
      Otto.sing(S_cuddly);
      break;
    case 7: //K 7
      Otto.sing(S_sleeping);
      break;
    case 8: //K 8
      Otto.sing(S_happy);
      break;
    case 9: //K 9
      Otto.sing(S_superHappy);
      break;
    case 10: //K 10
      Otto.sing(S_happy_short);
      break;
    case 11: //K 11
      Otto.sing(S_sad);
      break;
    case 12: //K 12
      Otto.sing(S_confused);
      break;
    case 13: //K 13
      Otto.sing(S_fart1);
      break;
    case 14: //K 14
      Otto.sing(S_fart2);
      break;
    case 15: //K 15
      Otto.sing(S_fart3);
      break;
    case 16: //K 16
      Otto.sing(S_mode1);
      break;
    case 17: //K 17
      Otto.sing(S_mode2);
      break;
    case 18: //K 18
      Otto.sing(S_mode3);
      break;
    case 19: //K 19
      Otto.sing(S_buttonPushed);
      break;
    default:
      break;
  }

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


//-- Function to send ultrasonic sensor measure (distance)
void requestDistance() {

  Otto.home();  //stop if necessary

  int distance = Otto.getDistance();
  Serial.print(F("&&"));
  Serial.print(F("D "));
  Serial.print(distance);
  Serial.println(F("%%"));
  Serial.flush();
}


//-- Function to send noise sensor measure
void requestNoise() {

  Otto.home();  //stop if necessary

  int microphone = Otto.getNoise(); //analogRead(PIN_NoiseSensor);
  Serial.print(F("&&"));
  Serial.print(F("N "));
  Serial.print(microphone);
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
  //Serial.print(readVcc());
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
      delay(300);
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
