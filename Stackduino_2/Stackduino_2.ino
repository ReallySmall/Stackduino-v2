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

int sliceSize = 10; //default number of sliceSize which stepper motor should make between sliceNum
int sliceNum = 10; //default total number of focus sliceNum to make
int sliceCounter = 0; //count of number of focus sliceNum made so far
int pause = 5000; //default time in millis to wait for camera to take picture
int encoderPos = 1; //which menu item to display when turning rotary encoder
int measure = 1; //whether to use microns, mm or cm when making sliceSize
int measureMultiplier = 1; //multiplier to factor into step signals (1, 1000 or 10,000) depending on unit of measure used
int stepDelay = 2000; //delay in microseconds between motor steps, governing motor speed in stack
int sliceBracket = 1; //number of images to bracket per focus slice
int encoderCount = 0; //count pulses from encoder
boolean disableA4988 = true; //whether to disable easydriver betweem sliceSize to save power and heat
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
    0,1,2,3,4,5,6          };
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
    8,9,10,11,12,13,14,15          };
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
  //CHECK LIMIT SWITCHES
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  clearLimitSwitch();

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
      menuChange(encoderPos, 0, 8, 1); 
    } 

    switch (encoderPos) { //the menu options

    case 1: //this menu screen changes the number of increments to move each time

      menuChange(sliceSize, 1, 1000, 1);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Set slice size:  ");
        screen.setLCDColRow(0, 1);
        frontLoadAndPrint(sliceSize); //frontload with correct number of zeroes and print to screen
        screen.print (sliceSize  , DEC);
        unitOfMeasure();
        updateScreen = false;

      }      

      break;

    case 2: //this menu screen changes the number of sliceNum to create in the stack

      menuChange(sliceNum, 10, 5000, 10);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Number of sliceNum: ");
        screen.setLCDColRow(0, 1);
        frontLoadAndPrint(sliceNum); //then use that figure to frontload with correct number of zeroes and print to screen
        screen.print (sliceNum, DEC);
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

    case 5: //this menu screen selects the unit of measure to use for sliceSize: Microns, Millimimeters or Centimeteres

      menuChange(measure, 1, 3, 1);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Unit of measure:");
        screen.setLCDColRow(0, 1);
        unitOfMeasure();

        updateScreen = false;

      }

      break; 

    case 6: //this menu screen adjusts the stepper motor speed (delay in microseconds between sliceSize)
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

      menuChange(sliceBracket, 1, 10, 1);

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.setLCDColRow(0, 0);
        screen.print("Set bracketing: ");
        screen.setLCDColRow(0, 1);
        frontLoadAndPrint(sliceBracket); //then use that figure to frontload with correct number of zeroes and print to screen
        screen.print (sliceBracket  , DEC);           
        updateScreen = false;

      }      

      break;

    }
  } //end of setup menu section

  else { 

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //THE FOCUS STACK, STARTED WHEN THE MAIN PUSHBUTTON IS PRESSED. THE LOOP ADVANCES THE CAMERA, TAKES A PICTURE AND REPEATS.
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////      

    disableStepperDriver(); //disable the stepper driver when not in use to save power

      for (int i = 0; i < sliceNum; i++){ //loop the following actions for number of times dictated by var sliceNum

      sliceCounter++; //count of pictures taken so far
      if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 
      screen.clearScreen();
      screen.print("Moving ");
      screen.print (sliceSize);
      unitOfMeasure();
      screen.setLCDColRow(0, 1);
      screen.print("Focus slice ");
      screen.print (sliceCounter);
      screen.print (" of ");
      screen.print (sliceNum);
      if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 
      enableStepperDriver();
      digitalWrite(dir, LOW); //set the stepper direction for forward travel
      delay(100);

      for (int i = 0; i < sliceSize * 16 * measureMultiplier; i++){ ////adjust the number in this statement to tune distance travelled on your setup. In this case 16 is a product of 8x microstepping and a 2:1 gearing ratio
        stepSignal(); //send a step signal to the stepper driver
        if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
          break;
        } 
      }
  
      if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 
      disableStepperDriver(); //disable the stepper driver when not in use to save power  
      takePicture(); //send signal to camera to take picture

    }

    screen.setLCDColRow(0, 0);
    screen.print("Stack finished");
    delay(2000);
    screen.clearScreen(); 

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //THE FOCUS STACK HAS FINISHED. IF THE OPTION IS ENABLED THE CAMERA STAGE IS RETURNED TO ITS START POSITION.
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (returnToStart){   
      digitalWrite(dir, HIGH); //set the stepper direction for backward travel (if your motor is wired the other way around this you may need to reverse this)
      screen.setLCDColRow(0, 0);
      screen.print("<< Returning"); 
      int returnSteps = sliceSize * sliceCounter;
      screen.setLCDColRow(0, 1);
      screen.print (returnSteps);
      unitOfMeasure();

      enableStepperDriver();

      for (int i; i < returnSteps * 16 * measureMultiplier; i++){
        stepSignal();
        if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
          break;
        } 
      }

      disableStepperDriver(); 
      screen.clearScreen();
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //RUN CLEANUP ON VARIABLES, HANDLE ANY CALLS TO CANCEL STACK EARLY, THEN GO BACK TO MENU SECTION
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    stackEnd();

  } 
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//FUNCTIONS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/* RETURN CURRENT STATE OF MAIN PUSH BUTTON */
void buttonChange(){ //function to read the current state of the push button

  detachInterrupt(1); //detach the interrupt while the current instance is dealt with

  reading = digitalRead(pushButton);

  if (reading == LOW && previous == HIGH && millis() - time > debounce) {
    if (buttonState == HIGH)
      buttonState = LOW;
    else
      buttonState = HIGH;

    time = millis();    
  }

  previous = reading;

  attachInterrupt(1, buttonChange, CHANGE); //main pushbutton on interrupt 1
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
    digitalWrite(dir, HIGH);
    for (int i = 0; i<1; i++)
    {
      stepSignal(); //move forward
    }
    disableStepperDriver();
  }

  while (mcp.digitalRead(backwardControl) == LOW) {
    enableStepperDriver();
    digitalWrite(dir, LOW);
    for (int i = 0; i<1; i++)
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
  delayMicroseconds(stepDelay); //delay time between sliceSize, too fast and motor stalls

}

/* SEND SIGNAL TO CAMERA TO TAKE PICTURE WITH DELAYS TO ALLOW SETTLING */
void takePicture(){
  for (int i = 1; i <= sliceBracket; i++){
    if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    } 
    if(i > 1){ //if more than one image is being taken, display the current sliceBracket number
      screen.print("Bracketed image:");
      screen.setLCDColRow(0, 1);
      screen.print(i);
      screen.print(" of ");
      screen.print(sliceBracket);
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

    //A delay is required here for the camera to take an image, taking into account mirror-lockup and flash recharge time
    //The controller should still respond to a cancel event during this time so a non-blocking delay is required
    unsigned long currentMillis = millis(); //get the current millis
    unsigned long endMillis = currentMillis + pause; //the time to finish the delay

    while(endMillis > millis()){
      if(cancelStack()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 
    }

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
      encoderPos++;
      updateScreen = true; //flag that a menu has changed so screen updates with new value
      encoderCount = 0;
    }
    if(tmpdata == -1){
      encoderCount--;
    }
    if(encoderCount == -2){ //change this number to adjust encoder sensitivity
      encoderPos--;
      updateScreen = true; //flag that a menu has changed so screen updates with new value
      encoderCount = 0;
    }
  }
  return encoderPos;
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
  screen.clearScreen();
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
  
  if(buttonState == LOW){ //if a stack was in progress, cancel it as there probably isn't space to continue
buttonState = HIGH;
cancelStack();
  }
}

/* PROCESS INTERRUPTS FROM THE MCP23017 */
void handleInterrupt(){

  //find out which pin on the mcp created the interrupt and its current state
  uint8_t pin=mcp.getLastInterruptPin();
  uint8_t val=mcp.getLastInterruptPinValue();

  detachInterrupt(0); //detach the interrupt while the current instance is dealt with

  //if a switchoff signal was recieved from the pushbutton controller pull kill low to power off
  if(pin == pbInterrupt && val == LOW) {
    mcp.digitalWrite(kill, LOW);
  }

  //if a limit switch was tripped call the function to clear it
  if((pin == limitSwitchFront || pin == limitSwitchBack) && val == LOW){
    clearLimitSwitch();
  }

  attachInterrupt(0,handleInterrupt,FALLING); //re-attach the interrupt

}

/* RUN CLEANUP ON VARIABLES THEN GO BACK TO MENU SECTION */
void stackEnd(){

  encoderPos = 1; //set menu to first option screen
  sliceCounter = 0; //reset pic counter
  buttonState = HIGH; //return to menu options section

}

/* CHECK IF THE MAIN BUTTON HAS BEEN PUSHED DURING STACK AND CANCEL IF YES */
boolean cancelStack(){

  if(buttonState == HIGH){
    screen.clearScreen();
    screen.setLCDColRow(0, 0);
    screen.print("Stack cancelled");
    delay(1000);
    screen.clearScreen();
    
    return true;
  } else {
    return false; 
  }

}
