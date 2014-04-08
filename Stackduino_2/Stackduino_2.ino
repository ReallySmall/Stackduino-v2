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

int mcpInt=2; //MCP23017 interrupt
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
int kill = 10; //LTC2950 kill pin - disable 5v regulator and switch off controller
int MS1 = 11; //toggle A4988 stepper driver MS1 pin to control degree of microstepping
int MS2 = 12; //toggle A4988 stepper driver MS2 pin to control degree of microstepping
int MS3 = 13; //toggle A4988 stepper driver MS3 pin to control degree of microstepping
int enable = 14; //disable/enable A4988 stepper driver to conserve power
int sleep = 15; //sleep/wake A4988 stepper driver to conserve power

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GLOBAL VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int steps = 10; //default number of steps which stepper motor should make between slices
int slices = 10; //default total number of focus slices to make
int sliceCount = 0; //count of number of focus slices made so far
int pause = 5000; //default time in millis to wait for camera to take picture
int rotaryCount = 1; //which menu item to display when turning rotary encoder
int measure = 1; //whether to use microns, mm or cm when making steps
int measureMultiplier = 1; //multiplier to factor into step signals (1, 1000 or 10,000) depending on unit of measure used
int stepDelay = 2000; //delay in microseconds between motor steps, governing motor speed in stack
int bracket = 1; //number of images to bracket per focus slice
int encoderCount = 0; //count pulses from encoder
boolean disableA4988 = true; //whether to disable easydriver betweem steps to save power and heat
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //SET ATMEGA PINMODES AND PULLUPS
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pinMode(pushButton, INPUT); 
  digitalWrite(pushButton, HIGH);
  pinMode(ENC_A, INPUT); 
  digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT); 
  digitalWrite(ENC_B, HIGH);
  pinMode(dir, OUTPUT); 
  digitalWrite(dir, LOW); 
  pinMode(doStep, OUTPUT); 
  digitalWrite(doStep, LOW);     
  pinMode(focus, OUTPUT); 
  digitalWrite(focus, LOW);
  pinMode(shutter, OUTPUT); 
  digitalWrite(shutter, LOW);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //START MCP23017 AND SET PINMODES AND PULLUPS
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  mcp.begin();// using default address 0
  mcp.setupInterrupts(true,false,LOW);

  //set MSCP23017 bank A pins as inputs and switch on internal pullups
  //lots of pins with the same settings so quicker to loop through with array
  int mcpInputPins[] = {
    0,1,2,3,4,5,6      };
  for(int i=0; i < 7; i++){
    mcp.pinMode(mcpInputPins[i], INPUT);
    mcp.pullUp(mcpInputPins[i], HIGH);
  }

  mcp.setupInterrupts(true,false,LOW);
  mcp.setupInterruptPin(limitSwitchFront,FALLING);
  mcp.setupInterruptPin(limitSwitchBack,FALLING);
  mcp.setupInterruptPin(pbInterrupt,FALLING);

  //set MSCP23017 bank B pins as outputs and write HIGH
  //lots of pins with the same settings so quicker to loop through with array
  int mcpOutputPins[] = {
    8,9,10,11,12,13,14,15      };
  for(int i=0; i < 8; i++){
    mcp.pinMode(mcpOutputPins[i], OUTPUT);
    mcp.digitalWrite(mcpOutputPins[i], HIGH);
  }
  //enable is an exception and needs to be overwritten to LOW  
  mcp.digitalWrite(enable, LOW);

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //START SERIAL CONNECTION AND ATTACH INTERRUPTS
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.begin(9600);  

  attachInterrupt(0,handleInterrupt,FALLING); //mcp23017 on interrupt 0
  attachInterrupt(1, buttonChange, CHANGE); //main pushbutton on interrupt 1

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //CHECK LIMIT SWITCHES AND PRINT WELCOME MESSAGE TO SCREEN
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  
  clearLimitSwitch();
  screen.setLCDColRow(0, 0);
  screen.print("Stackduino");
  delay(1000);
  screen.clearScreen();

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MAIN LOOP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void loop(){

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //SETTINGS CONFIGURATION AND MANUAL STAGE CONTROL
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  if (buttonState == HIGH){ //this section allows manual control and configures settings using a simple screen menu system

    disableStepperDriver(); //switch off stepper motor power if option enabled
    manualControl(); //manual motor control to position stage before stack

    if (rbbuttonState == HIGH) { //use encoder to scroll through menu options
      menuChange(rotaryCount, 0, 8, 1); 
    } 

    switch (rotaryCount) { //the menu options

    case 1: //this menu screen changes the number of steps to move each time

      menuChange(steps, 1, 1000, 1);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Set step size:  ");
        screen.setLCDColRow(0, 1);
        frontLoadAndPrint(steps); //frontload with correct number of zeroes and print to screen
        screen.print (steps  , DEC);
        unitOfMeasure();
        updateScreen = false;

      }      

      break;

    case 2: //this menu screen changes the number of slices to create in the stack

      menuChange(slices, 10, 5000, 10);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Set num slices: ");
        screen.setLCDColRow(0, 1);
        frontLoadAndPrint(slices); //then use that figure to frontload with correct number of zeroes and print to screen
        screen.print (slices, DEC);
        updateScreen = false;

      } 

      break;

    case 3: //this menu screen changes the number of seconds to wait for the camera to take a picture before moving again - 
      //you may want longer if using flashes for adequate recharge time or shorter with continuous lighting
      //to reduce overall time taken to complete the stack

      menuChange(pause, 1000, 30000, 1000);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Set pause time: ");
        screen.setLCDColRow(0, 1);
        if (pause < 10000){
          screen.print (0, DEC); //adds one leading zero to triple digit pause numbers on the display
        }
        screen.print ((pause / 1000), DEC); //divide millis by 1000 to display in seconds
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

      menuChange(measure, 1, 3, 1);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Unit of measure:");
        screen.setLCDColRow(0, 1);
        unitOfMeasure();

        updateScreen = false;

      }

      break; 

    case 6: //this menu screen adjusts the stepper motor speed (delay in microseconds between steps)
      //setting this to low may cause the motor to begin stalling or failing to move at all

      menuChange(stepDelay, 1000, 9000, 1000);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Stepper speed: ");
        screen.setLCDColRow(0, 1);
        screen.print (stepDelay, DEC);
        screen.print (" microsecs");
        updateScreen = false;

      }

      break; 

    case 7: //this menu screen changes the number of images to take per focus slice (exposure bracketing support)

      menuChange(bracket, 1, 10, 1);

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

  else { 

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //THE FOCUS STACK, STARTED WHEN THE MAIN PUSHBUTTON IS PRESSED
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

    disableStepperDriver();

    for (int i = 0; i < slices; i++){ //loop the following actions for number of times dictated by var slices

      sliceCount++; //count of pictures taken so far

      screen.clearScreen();
      screen.print("Moving ");
      screen.print (steps);
      unitOfMeasure();
      screen.setLCDColRow(0, 1);
      screen.print("Step ");
      screen.print (sliceCount);
      screen.print (" of ");
      screen.print (slices);

      delay(500);
      enableStepperDriver();
      digitalWrite(dir, HIGH); //set the stepper direction for backward travel (if your motor is wired the other way around you may need to reverse this)
      delay(100);

      int i = 0; //counter for motor steps
      while (i < steps * 16 * measureMultiplier){ //adjust the number in this statement to tune distance travelled on your setup. In this case 16 is a product of 8x microstepping and a 2:1 gearing ratio
        stepSignal();
        i++;
      }
      i = 0; //reset counter
      disableStepperDriver();   

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
      int returnSteps = steps * sliceCount;
      screen.setLCDColRow(0, 1);
      screen.print (returnSteps);
      unitOfMeasure();

      enableStepperDriver();

      int i = 0; //counter for motor steps
      while (i < returnSteps * 16 * measureMultiplier){

        stepSignal();
        i++;
      }
      i = 0; //reset counter

      disableStepperDriver(); 
      screen.clearScreen();
    }
    rotaryCount = 1; //set menu option display to first
    sliceCount = 0; //reset pic counter
    buttonState = HIGH; //return to menu options
  } 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

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

  rbreading = mcp.digitalRead(rotarypushButton);
  if (rbreading == LOW && rbprevious == HIGH && millis() - rbtime > rbdebounce) {
    if (rbbuttonState == HIGH)
      rbbuttonState = LOW;
    else
      rbbuttonState = HIGH;
    rbtime = millis();    
  }

  rbprevious = rbreading;

} 

/* MOVE CARRIAGE BACKWARD AND FORWARD USING PUSH BUTTONS */
void manualControl(){

  while (mcp.digitalRead(forwardControl) == LOW) {
    enableStepperDriver();
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
    disableStepperDriver();
  }

  while (mcp.digitalRead(backwardControl) == LOW) {
    enableStepperDriver();
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
    disableStepperDriver();
  }

}

/* SEND STEP SIGNAL TO EASYDRIVER TO TURN MOTOR */
void stepSignal(){

  digitalWrite(doStep, LOW); //this LOW to HIGH change is what creates the
  digitalWrite(doStep, HIGH); //"Rising Edge" so the easydriver knows to when to step
  delayMicroseconds(stepDelay); //delay time between steps, too fast and motor stalls

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
    screen.print ((pause / 1000), DEC);
    screen.print(" seconds)");

    digitalWrite(focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(shutter, HIGH); //trigger camera shutter

    delay(200); //small delay needed for camera to process above signals

    digitalWrite(shutter, LOW); //switch off camera trigger signal
    digitalWrite(focus, LOW); //switch off camera focus signal

    delay(pause); //pause to allow for camera to take picture with mirror lockup and to allow flashes to recharge before next shot

    screen.clearScreen();
  }
}

/* ENABLE THE STEPPER DRIVER */
void enableStepperDriver() {
  mcp.digitalWrite(enable, LOW);
}

/* DISABLE THE STEPPER DRIVER WHEN NOT IN USE IF OPTION SET */
void disableStepperDriver() {
  if(disableA4988) {
    mcp.digitalWrite(enable, HIGH);
  } 
  else {
    mcp.digitalWrite(enable, LOW);
  } 
}

/* PRINT SELECTED UNIT OF MEASURE TO SCREEN */
void unitOfMeasure() {        
  if (measure==1){
    measureMultiplier = 1;
    screen.print(" mn");
  }
  if (measure==2){
    measureMultiplier = 1000;
    screen.print(" mm");
  }
  if (measure==3){
    measureMultiplier = 10000;
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

/* MONITOR THE ROTARY ENCODER AND PROCESS ANY MENU SCREEN OR ITEM CHANGES */

void menuChange(int &menuvar, int constrainLow, int constrainHigh, int multiplier) {

  //If the encoder is in menu screen changing mode
  if (rbbuttonState == HIGH) { //pressing rotary encoder button enables editing of selected menu item's variable
    menuvar = constrain(menuvar, constrainLow, constrainHigh); //limits choice of input step size to specified range
    menuvar += rotaryCountRead();  //use encoder reading function to set value of variable
    if (menuvar == 8){ //when counter value exceeds number of menu items
      menuvar = 1; //reset it to 1 again to create a looping navigation
    }
    if (menuvar == 0){ //when counter value goes below minimum number of menu items
      menuvar = 7; //reset it to 7 again to create a looping navigation
    } 
  }

  //If the encoder is in menu item changing mode within a menu screen
  if (rbbuttonState == LOW) { //pressing rotary encoder button enables editing of selected menu item's variable
    menuvar = constrain(menuvar, constrainLow, constrainHigh); //limits choice of input step size to specified range
    menuvar += rotaryCountRead() * multiplier;  //use encoder reading function to set value of variable
  }

}

/* FILTER OUTPUT FROM THE ROTARY ENCODER FUNCTION AND RETURN CURRENT VALUE */

int rotaryCountRead(){

  int8_t tmpdata = read_encoder(); //counts encoder pulses and registers only every nth pulse to adjust feedback sensitivity

  if(tmpdata){
    if(tmpdata == 1){
      encoderCount++;
    }
    if(encoderCount == 2){ //change this number to adjust encoder sensitivity
      rotaryCount++;
      updateScreen = true; //flag that a menu has changed so screen updates with new value
      encoderCount = 0;
    }
    if(tmpdata == -1){
      encoderCount--;
    }
    if(encoderCount == -2){ //change this number to adjust encoder sensitivity
      rotaryCount--;
      updateScreen = true; //flag that a menu has changed so screen updates with new value
      encoderCount = 0;
    }
  }
  return rotaryCount;
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

/* PULL CARRIAGE AWAY FROM TRIPPED LIMIT SWITCH */
void clearLimitSwitch(){
  enableStepperDriver();
  screen.setLCDColRow(0, 0);
  screen.print("End of travel!  ");
  screen.setLCDColRow(0, 1);
  screen.print("Returning...    ");

  if (mcp.digitalRead(limitSwitchFront) == LOW){
    digitalWrite(dir, LOW); //reverse stepper motor direction
    while (mcp.digitalRead(limitSwitchFront) == LOW) //iterate doStep signal for as long as  the limit switch remains pressed 
    {  
      stepSignal();
    }
    digitalWrite(dir, HIGH); //restore normal stepper motor direction
  }

  if (mcp.digitalRead(limitSwitchBack) == LOW){
    digitalWrite(dir, HIGH); //reverse stepper motor direction
    while (mcp.digitalRead(limitSwitchBack) == LOW) //iterate doStep signal for as long as  the limit switch remains pressed 
    {  
      stepSignal();
    }
    digitalWrite(dir, LOW); //restore normal stepper motor direction
  }

  screen.clearScreen();
  disableStepperDriver();
}

/* PROCESS INTERRUPTS FROM THE MCP23017 */
void handleInterrupt(){

  //find out which pin on the mcp created the interrupt and its current state
  uint8_t pin=mcp.getLastInterruptPin();
  uint8_t val=mcp.getLastInterruptPinValue();

  detachInterrupt(0); //detach the interrupt while the current instance is dealt with

  //if a limit switch was tripped call the function to clear it
  if((pin == limitSwitchFront || pin == limitSwitchBack) && val == LOW){
    clearLimitSwitch();
  }
  //if a switchoff signal was recieved from the pushbutton controller pull kill low to power off
  if(pin == pbInterrupt && val == LOW) {
    mcp.digitalWrite(kill, LOW);
  }

  attachInterrupt(0,handleInterrupt,FALLING); //re-attach the interrupt

}




