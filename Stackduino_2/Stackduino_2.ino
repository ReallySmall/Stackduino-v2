/*
 STACKDUINO 2
 
 A sketch to drive an Arduino compatible MacroPhotography Focus Stacking Controller
 
 */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//INCLUDE LIBRARIES
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define _Digole_Serial_SPI_
#include <Wire.h>
#include "Adafruit_MCP23017.h" //customised version of Adafruit library with interrupt functionality by husio-org: https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/pull/6
#include <DigoleSerial.h> //https://github.com/chouckz/HVACX10Arduino/blob/master/1.0.1_libraries/DigoleSerial/DigoleSerial.h

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//INSTANTIATE OBJECTS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Adafruit_MCP23017 mcp; //port expander
DigoleSerialDisp screen(8,9,10);  //OLED screen - SPI mode - Pin 8: data, 9:clock, 10: SS

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEFINE REMAINING ATMEGA PINS
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//rotary encoder
#define ENC_A A0
#define ENC_B A1
#define ENC_PORT PINC

int arduinoIntPin=2; //MCP23017 interrupt
int pushButton = 3;  //start/ stop stack button
int dir = 4; //direction pin on A4988 stepper driver
int doStep = 5; //step pin on A4988 stepper driver
int focus = 6; //camera autofocus
int shutter = 7; //camera shutter
int analogueExt1 = A2; //analogue pin broken out through DB15 - unused (for future functionality)
int analogueExt2 = A3; //analogue pin broken out through DB15 - unused (for future functionality)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEFINE MCP23017 PINS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//inputs
int forwardControl = 0; //for manual positioning
int backwardControl = 1; //for manual positioning
int limitSwitchFront = 2; //limit switches to stop stepping if end of travel on rail is reached at either end
int limitSwitchBack = 3; //limit switches to stop stepping if end of travel on rail is reached at either end
int stat = 4; //LTC4412 stat pin - indicate whether controller is currently powered by wall adapter or battery
int rotarypushButton = 5; //select/ unselect menu item button
int pbInterrupt = 6; //LTC2950 interrupt pin - signal to the controller that it is about to be switched off
//pin 7 NC

//outputs
int digitalExt1 = 8; //digital pin broken out through DB15 - unused (for future functionality)
int digitalExt2 = 9;//digital pin broken out through DB15 - unused (for future functionality)
int Kill = 10; //LTC2950 kill pin - disable 5v regulator and switch off controller
int MS1 = 11; //toggle A4988 stepper driver MS1 pin to control degree of microstepping
int MS2 = 12; //toggle A4988 stepper driver MS2 pin to control degree of microstepping
int MS3 = 13; //toggle A4988 stepper driver MS3 pin to control degree of microstepping
int enable = 14; //disable/enable A4988 stepper driver to conserve power
int Sleep = 15; //sleep/wake A4988 stepper driver to conserve power

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GLOBAL VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int steps = 10; //default number of microns stepper motor should make between pictures
int numPictures = 10; //default total number of pictures to take
int picCounter = 0; //count of number of pictures taken so far
int setPause = 5000; //default time in millis to wait for camera to take picture
int manualSpeedXaxis = 0; //delay between steps in manual mode, governing motor speed
int rotaryCounter = 1; //which menu item to display when turning rotary encoder
int unitofMeasure = 1; //whether to use microns, mm or cm when making steps
int uomMultiplier = 1; //multiplier to factor into step signals (1, 1000 or 10,000) depending on unit of measure used
int stepSpeed = 2000; //delay in microseconds between motor steps, governing motor speed in stack
int bracket = 1; //number of images to bracket per focus slice
int screenloopCounter = 0; //count number of loops to periodically update screen
int encoderCounter = 0; //count pulses from encoder
int varSize = 0; //used for frontloading menu item numbers with 0's
boolean disableEasydriver = true; //whether to disable easydriver betweem steps to save power and heat
boolean reverseFwdBwd = false; //change to true to reverse direction of the forward and backward manual control buttons
boolean returnToStart = false; //whether camera/ subject is returned to starting position at end of stack
boolean updateScreen = true; //flag true on startup and whenever encoder or button functions are called, prompting a screen update

//pushButton toggle
volatile int buttonState = HIGH; //the current state of the output pin
volatile int reading; //the current reading from the input pin
volatile int previous = LOW; //the previous reading from the input pin
volatile long time = 0; //the last time the output pin was toggled
volatile long debounce = 400; //the debounce time, increase if the output flickers

//rotary pushButton toggle
volatile int rbbuttonState = HIGH; //the current state of the output pin
volatile int rbreading; //the current reading from the input pin
volatile int rbprevious = LOW; //the previous reading from the input pin
volatile long rbtime = 0; //the last time the output pin was toggled
volatile long rbdebounce = 400; //the debounce time, increase if the output flickers

void setup() 

{

  Serial.begin(9600);  

  //Interrupt pins
  attachInterrupt(0,intCallBack,FALLING); //mcp23017 on interrupt 0
  attachInterrupt(1, buttonChange, CHANGE); //main pushbutton on interrupt 1
  
  mcp.begin();// using default address 0
  mcp.setupInterrupts(true,false,LOW);

  pinMode(pushButton, INPUT);
  pinMode(rotarypushButton, INPUT);
  pinMode(ENC_A, INPUT);
  pinMode(ENC_B, INPUT);
  pinMode(dir, OUTPUT);   
  pinMode(doStep, OUTPUT);    
  pinMode(focus, OUTPUT); 
  pinMode(shutter, OUTPUT); 
  pinMode(forwardControl, INPUT); 
  pinMode(backwardControl, INPUT); 
  pinMode(enable, OUTPUT);
  //pinMode(limitSwitches, INPUT);

  digitalWrite(focus, LOW);
  digitalWrite(shutter, LOW);
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  digitalWrite(pushButton, HIGH);
  digitalWrite(rotarypushButton, HIGH);
  digitalWrite(forwardControl, HIGH);
  digitalWrite(backwardControl, HIGH);
  digitalWrite(enable, LOW);
  //digitalWrite(limitSwitches, HIGH);

  screen.setLCDColRow(0, 0);
  screen.print("Stackduino");
  delay(1000);
  screen.clearScreen();

}

void loop(){

  if (digitalRead(limitSwitchFront) == LOW || digitalRead(limitSwitchBack) == LOW){ //if controller is started up hitting a limit switch disable main functions and print warning to screen

  }

  { //if limit switches HIGH run all functions normally

    if (buttonState == HIGH){ //this section allows manual control and configures settings using a simple screen menu system

      disableEasyDriver(); //switch off stepper motor power if option enabled
      manualControl(); //manual motor control to position stage before stack

      if (rbbuttonState == HIGH) { //use encoder to scroll through menu options
        rotaryCounterRead(); 

        rotaryCounter = constrain(rotaryCounter, 0, 8); //limits choice to specified range

        if (rotaryCounter == 8){ //when counter value exceeds number of menu items
          rotaryCounter = 1; //reset it to 1 again to create a looping navigation
        }

        if (rotaryCounter == 0){ //when counter value goes below minimum number of menu items
          rotaryCounter = 7; //reset it to 7 again to create a looping navigation
        }    
      } 

      switch (rotaryCounter) { //the menu options

      case 1: //this menu screen changes the number of steps to move each time

        menuVarChange(steps, 1, 1000, 1);

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Set step size:  ");
          screen.setLCDColRow(0, 1);
          varSize = numPictures; //transfer the value of the menu variable to varSize
          frontLoadAndPrint(steps); //frontload with correct number of zeroes and print to screen
          screen.print (steps  , DEC);
          unitOfMeasure();
          updateScreen = false;

        }      

        break;

      case 2: //this menu screen changes the number of pictures to take in the stack

        menuVarChange(numPictures, 10, 5000, 10);

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Set num steps: ");
          screen.setLCDColRow(0, 1);
          frontLoadAndPrint(numPictures); //then use that figure to frontload with correct number of zeroes and print to screen
          screen.print (numPictures, DEC);
          updateScreen = false;

        } 

        break;

      case 3: //this menu screen changes the number of seconds to wait for the camera to take a picture before moving again - 
        //you may want longer if using flashes for adequate recharge time or shorter with continuous lighting
        //to reduce overall time taken to complete the stack

        menuVarChange(setPause, 1000, 30000, 1000);

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Set pause time: ");
          screen.setLCDColRow(0, 1);
          if (setPause < 10000){
            screen.print (0, DEC); //adds one leading zero to triple digit setPause numbers on the display
          }
          screen.print ((setPause / 1000), DEC); //divide millis by 1000 to display in seconds
          screen.print(" seconds");  
          updateScreen = false;

        }

        break;

      case 4: //toggles whether camera/subject is returned the starting position at the end of the stack

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          returnToStart = constrain(returnToStart, true, false); //limits choice of returnToStart to specified range
          returnToStart += read_encoder (); //use encoder reading function to set value of returnToStart variable
        }

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Return to start:");
          screen.setLCDColRow(0, 1);
          if(returnToStart == true){
            screen.print ("Enabled");
          }
          else {
            screen.print ("Disabled");
          }
          updateScreen = false;

        }

        break; 

      case 5: //this menu screen selects the unit of measure to use for steps: Microns, Millimimeters or Centimeteres

        menuVarChange(unitofMeasure, 1, 3, 1);

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Unit of measure:");
          screen.setLCDColRow(0, 1);

          switch (unitofMeasure){ 

          case 1:

            screen.print ("Microns (mn)    ");
            uomMultiplier = 1;
            break;

          case 2:

            screen.print ("Millimetres (mm)");
            uomMultiplier = 1000;
            break;

          case 3: 

            screen.print ("Centimetres (cm)");
            uomMultiplier = 10000;
            break;
          }

          updateScreen = false;

        }

        break; 

      case 6: //this menu screen adjusts the stepper motor speed (delay in microseconds between steps)
        //setting this to low may cause the motor to begin stalling or failing to move at all

        menuVarChange(stepSpeed, 1000, 9000, 1000);

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Stepper speed: ");
          screen.setLCDColRow(0, 1);
          screen.print (stepSpeed, DEC);
          screen.print (" microsecs");
          updateScreen = false;

        }

        break; 

      case 7: //this menu screen changes the number of images to take per focus slice (exposure bracketing support)

        menuVarChange(bracket, 1, 10, 1);

        if (updateScreen){ //only write to the screen when a change to the variables has been flagged

          screen.setLCDColRow(0, 0);
          screen.print("Set bracketing: ");
          screen.setLCDColRow(0, 1);
          frontLoadAndPrint(bracket); //then use that figure to frontload with correct number of zeroes and print to screen
          screen.print (bracket  , DEC);           
          updateScreen = false;

        }      

        break;

      }
    } //end of setup menu section

    else { //this section runs the stack when the pushButton is pressed

      disableEasyDriver();

      for (int i = 0; i < numPictures; i++){ //loop the following actions for number of times dictated by var numPictures

        picCounter++; //count of pictures taken so far

        screen.clearScreen();
        screen.print("Moving ");
        screen.print (steps);
        unitOfMeasure();
        screen.setLCDColRow(0, 1);
        screen.print("Step ");
        screen.print (picCounter);
        screen.print (" of ");
        screen.print (numPictures);

        delay(500);
        enableEasyDriver();
        digitalWrite(dir, HIGH); //set the stepper direction for backward travel (if your motor is wired the other way around you may need to reverse this)
        delay(100);

        int i = 0; //counter for motor steps
        while (i < steps * 16 * uomMultiplier && digitalRead(limitSwitchFront) == HIGH && digitalRead(limitSwitchBack) == HIGH){ //adjust the number in this statement to tune distance travelled on your setup. In this case 16 is a product of 8x microstepping and a 2:1 gearing ratio
          stepSignal();
          i++;
        }
        i = 0; //reset counter
        disableEasyDriver();

        if (digitalRead(limitSwitchFront) == LOW || digitalRead(limitSwitchBack) == LOW){ //stop motor and reverse if limit switch hit
          retreat();
          break;
        }    

        if (buttonState == HIGH){ //if the Start/Stop stack button has been pressed, stop the stack even if not complete
          break;
        }

        takePicture(); //send signal to camera to take picture

          if (buttonState == HIGH){ //if the Start/Stop stack button has been pressed, stop the stack even if not complete
          break;
        }
      } 
      screen.setLCDColRow(0, 0);
      screen.print("Stack finished");
      delay(2000);
      screen.clearScreen(); 
      if (returnToStart == 1){   
        digitalWrite(dir, LOW); //set the stepper direction for backward travel (if your motor is wired the other way around this you may need to reverse this)
        delay(100);
        screen.setLCDColRow(0, 0);
        screen.print("<< Returning..."); 
        int returnSteps = steps * picCounter;
        screen.setLCDColRow(0, 1);
        screen.print (returnSteps);
        unitOfMeasure();

        enableEasyDriver();

        int i = 0; //counter for motor steps
        while (i < returnSteps * 16 * uomMultiplier && digitalRead(limitSwitchFront) == HIGH && digitalRead(limitSwitchBack) == HIGH){

          stepSignal();
          i++;
        }
        i = 0; //reset counter

        disableEasyDriver();

        if (digitalRead(limitSwitchFront) == LOW || digitalRead(limitSwitchBack) == LOW){ //stop motor and reverse if limit switch hit
          retreat();
        }  

        screen.clearScreen();
      }
      rotaryCounter = 1; //set menu option display to first
      picCounter = 0; //reset pic counter
      buttonState = HIGH; //return to menu options
    } 
  }
}

/* RETURN CURRENT STATE OF MAIN PUSH BUTTON */
void buttonChange(){ //function to read the current state of the push button

  reading = digitalRead(pushButton);

  if (reading == LOW && previous == HIGH && millis() - time > debounce) {
    if (buttonState == HIGH)
      buttonState = LOW;
    else
      buttonState = HIGH;

    time = millis();    
  }

  previous = reading;
} 

/* RETURN CURRENT STATE OF ROTARY ENCODER'S PUSH BUTTON */
void rotarybuttonChange(){

  rbreading = digitalRead(rotarypushButton);
  if (rbreading == LOW && rbprevious == HIGH && millis() - rbtime > rbdebounce) {
    if (rbbuttonState == HIGH)
      rbbuttonState = LOW;
    else
      rbbuttonState = HIGH;
    rbtime = millis();    
  }

  rbprevious = rbreading;

} 

/* PULL CARRIAGE BACK FROM TRIPPED LIMIT SWITCH */
void retreat(){
  enableEasyDriver();
  screen.setLCDColRow(0, 0);
  screen.print("End of travel!  ");
  screen.setLCDColRow(0, 1);
  screen.print("Reversing...    ");

  //while (digitalRead(limitSwitches) == LOW) //iterate doStep signal for as long as eitherlimit switch remains pressed 
  //{  
    //stepSignal();
  //}

  //digitalToggle(dir); //reset motor back to original direction once limit switch is no longer pressed
  screen.clearScreen();
  disableEasyDriver();
}

/* MOVE CARRIAGE BACKWARD AND FORWARD USING PUSH BUTTONS */
void manualControl(){

  while (digitalRead(forwardControl) == LOW) {
    enableEasyDriver();
    int i;
    if(reverseFwdBwd){
      digitalWrite(dir, LOW);
    } 
    else {
      digitalWrite(dir, HIGH);
    }
    for (i = 0; i<1; i++)
    {
      stepSignal(); //move forward
    }
    disableEasyDriver();
  }

  while (digitalRead(backwardControl) == LOW) {
    enableEasyDriver();
    int i;
    if(reverseFwdBwd){
      digitalWrite(dir, HIGH);
    } 
    else {
      digitalWrite(dir, LOW);
    } 
    for (i = 0; i<1; i++)
    {
      stepSignal(); //move backward
    }
    disableEasyDriver();
  }

}

/* SEND STEP SIGNAL TO EASYDRIVER TO TURN MOTOR */
void stepSignal(){

  digitalWrite(doStep, LOW); //this LOW to HIGH change is what creates the
  digitalWrite(doStep, HIGH); //"Rising Edge" so the easydriver knows to when to step
  delayMicroseconds(stepSpeed); //delay time between steps, too fast and motor stalls

}

/* SEND SIGNAL TO CAMERA TO TAKE PICTURE WITH DELAYS TO ALLOW SETTLING */
void takePicture(){
  for (int i = 1; i <= bracket; i++){
    screen.clearScreen();
    if(i > 1){ //if more than one image is being taken, display the current bracket number
      screen.print("Bracketed image:");
      screen.setLCDColRow(0, 1);
      screen.print(i);
      screen.print(" of ");
      screen.print(bracket);
      delay(1000);
    }
    screen.clearScreen();
    screen.print("Pause for image");
    screen.setLCDColRow(0, 1);
    screen.print("(");
    screen.print ((setPause / 1000), DEC);
    screen.print(" seconds)");

    digitalWrite(focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(shutter, HIGH); //trigger camera shutter

    delay(200); //small delay needed for camera to process above signals

    digitalWrite(shutter, LOW); //switch off camera trigger signal
    digitalWrite(focus, LOW); //switch off camera focus signal

    delay(setPause); //pause to allow for camera to take picture with mirror lockup and to allow flashes to recharge before next shot

    screen.clearScreen();
  }
}

/* ENABLE THE EASYDRIVER */
void enableEasyDriver() {
  digitalWrite(enable, LOW);
}

/* DISABLE THE EASYDRIVER WHEN NOT IN USE IF OPTION SET */
void disableEasyDriver() {
  if(disableEasydriver == true) {
    digitalWrite(enable, HIGH);
  } 
  else {
    digitalWrite(enable, LOW);
  } 
}

/* PRINT SELECTED UNIT OF MEASURE TO SCREEN */
void unitOfMeasure() {        
  if (unitofMeasure==1){
    screen.print(" mn");
  }
  if (unitofMeasure==2){
    screen.print(" mm");
  }
  if (unitofMeasure==3){
    screen.print(" cm");
  }
}

/* FRONT PAD MENU ITEM VARIABLE NUMBERS WITH ZEROES 
 Maintains consistent formatting */

void frontLoadAndPrint(int menuvar) {
  if (menuvar < 10){
    screen.print (000, DEC); //adds three leading zeros to single digit Step size numbers on the display
  }
  if (menuvar < 100){
    screen.print (00, DEC); //adds two leading zeros to double digit Step size numbers on the display
  }
  if (menuvar < 1000){
    screen.print (0, DEC); //adds one leading zero to triple digit Step size numbers on the display
  }
  screen.print(' ');
  screen.print(menuvar);
}

/* PROCESS A MENU ITEM CHANGE */

void menuVarChange(int &menuvar, int constrainLow, int constrainHigh, int multiplier) {

  if (rbbuttonState == LOW) { //pressing rotary encoder button enables editing of selected menu item's variable
    menuvar = constrain(menuvar, constrainLow, constrainHigh); //limits choice of input step size to specified range
    menuvar += rotaryCounterRead() * multiplier;  //use encoder reading function to set value of steps variable
  }

}

/* FILTER OUTPUT FROM THE ROTARY ENCODER AND SELECT THE ACTIVE MENU ITEM */

int rotaryCounterRead(){

  int8_t tmpdata = read_encoder(); //counts encoder pulses and registers only every nth pulse to adjust feedback sensitivity

  if(tmpdata){
    if(tmpdata == 1){
      encoderCounter++;
    }
    if(encoderCounter == 2){ //change this number to adjust encoder sensitivity
      rotaryCounter++;
      updateScreen = true; //flag that a menu has changed so screen updates with new value
      encoderCounter = 0;
    }
    if(tmpdata == -1){
      encoderCounter--;
    }
    if(encoderCounter == -2){ //change this number to adjust encoder sensitivity
      rotaryCounter--;
      updateScreen = true; //flag that a menu has changed so screen updates with new value
      encoderCounter = 0;
    }
  }
  return rotaryCounter;
}

/* RETURN CHANGE IN ENCODER STATE */
int8_t read_encoder(){

  static int8_t enc_states[] = { 
    //0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 //use this line instead to increment encoder in the opposite direction
    0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0
  };
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2; //remember previous state
  old_AB |= ( ENC_PORT & 0x03 ); //add current state

  return ( enc_states[( 0x0f & old_AB )]);

}

// The int handler will just signal that the int has happen
// we will do the work from the main loop.
void intCallBack(){
  
}

