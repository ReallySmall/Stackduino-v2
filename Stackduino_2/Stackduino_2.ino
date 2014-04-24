/*
STACKDUINO 2
 
 An Arduino compatible MacroPhotography Focus Stacking Controller.
 
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

const int mcpInt=2; //MCP23017 interrupt
const int pushButton = 3;  //start/ stop stack button
const int dir = 4; //direction pin on A4988 stepper driver
const int doStep = 5; //step pin on A4988 stepper driver
const int focus = 6; //camera autofocus
const int shutter = 7; //camera shutter
const int analogueExt1 = A2; //analogue pin broken out through DB15 - unused (for future functionality)
const int analogueExt2 = A3; //analogue pin broken out through DB15 - unused (for future functionality)

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//DEFINE MCP23017 PINS
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//inputs
const int forwardControl = 0; //for manual positioning
const int backwardControl = 1; //for manual positioning
const int limitSwitchFront = 2; //limit switches to stop stepping if end of travel on rail is reached at either end
const int limitSwitchBack = 3; //limit switches to stop stepping if end of travel on rail is reached at either end
const int stat = 4; //LTC4412 stat pin - indicate whether controller is currently powered by wall adapter or battery
const int rotarypushButton = 5; //select/ unselect menu item button
const int pbInterrupt = 6; //LTC2950 interrupt pin - signal to the controller that it is about to be switched off
//pin 7 NC

//outputs
const int digitalExt1 = 8; //digital pin broken out through DB15 - unused (for future functionality)
const int digitalExt2 = 9;//digital pin broken out through DB15 - unused (for future functionality)
const int kill = 10; //LTC2950 kill pin - disable 5v regulator and switch off controller
const int MS1 = 11; //toggle A4988 stepper driver MS1 pin to control degree of microstepping
const int MS2 = 12; //toggle A4988 stepper driver MS2 pin to control degree of microstepping
const int MS3 = 13; //toggle A4988 stepper driver MS3 pin to control degree of microstepping
const int enable = 14; //disable/enable A4988 stepper driver to conserve power
const int sleep = 15; //sleep/wake A4988 stepper driver to conserve power

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//GLOBAL VARIABLES
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

int sliceSize = 10; //default number of sliceSize which stepper motor should make between sliceNum
int sliceNum = 10; //default total number of focus sliceNum to make
int sliceCounter = 0; //count of number of focus sliceNum made so far
int pause = 1; //default time in seconds to wait for camera to take picture
int encoderPos = 1; //which menu item to display when turning rotary encoder
int lastEncoderPos; //used to check whether the encoder position has changed
int measure = 1; //whether to use microns, mm or cm when making sliceSize
int measureMultiplier = 1; //multiplier to factor into step signals (1, 1000 or 10,000) depending on unit of measure used
int stepDelay = 2000; //delay in microseconds between motor steps, governing motor speed in stack
int sliceBracket = 1; //number of images to bracket per focus slice
int encoderCount = 0; //count pulses from encoder
int returnToStart = 1; //whether camera/ subject is returned to starting position at end of stack
boolean disableA4988 = true; //whether to disable easydriver betweem sliceSize to save power and heat
boolean updateScreen = true; //flag true on startup and whenever encoder or button functions are called, prompting a screen update

//pushButton toggle
volatile int startStack = false; //the current state of the output pin
volatile int reading; //the current reading from the input pin
volatile int previous = LOW; //the previous reading from the input pin
volatile long time = 0; //the last time the output pin was toggled
volatile long debounce = 300; //the debounce time, increase if the output flickers

//rotary pushButton toggle
volatile int rbbuttonState = HIGH; //the current state of the output pin
volatile int rbreading; //the current reading from the input pin
volatile int rbprevious = LOW; //the previous reading from the input pin
volatile long rbtime = 0; //the last time the output pin was toggled
volatile long rbdebounce = 300; //the debounce time, increase if the output flickers

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//SETUP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //SET ATMEGA PINMODES AND PULLUPS
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  pinMode(mcpInt, INPUT);
  digitalWrite(mcpInt, HIGH);
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
    0,1,2,3,4,5,6                      };
    for(int i=0; i < 7; i++){
      mcp.pinMode(mcpInputPins[i], INPUT);
    if(i == 4 || i == 6){ //stat and pbInterrupt have external pullups so don't need setting HIGH
    continue;
  }
  mcp.pullUp(mcpInputPins[i], HIGH);
}

mcp.setupInterrupts(true,false,LOW);
mcp.setupInterruptPin(limitSwitchFront,FALLING);
mcp.setupInterruptPin(limitSwitchBack,FALLING);
mcp.setupInterruptPin(pbInterrupt,FALLING);
mcp.setupInterruptPin(rotarypushButton,FALLING);

  //set MSCP23017 bank B pins as outputs and write HIGH
  //lots of pins with the same settings so quicker to loop through with array
  int mcpOutputPins[] = {
    8,9,10,11,12,13,14,15};
    for(int i=0; i < 8; i++){
      mcp.pinMode(mcpOutputPins[i], OUTPUT);
      if(i == 14){
        mcp.digitalWrite(mcpOutputPins[i], LOW); //just to be awkward, enable needs setting LOW
        } else {
        mcp.digitalWrite(mcpOutputPins[i], HIGH); //but every other mcp output pin should be HIGH
      }
    }

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  //START SERIAL CONNECTION AND ATTACH INTERRUPTS
  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  Serial.begin(9600); //start serial
  screen.displayConfig(0); //disable screen config message
  screen.displayStartScreen(0); //disable splash screen

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

  if (startStack == false){ //this section allows manual control and configures settings using a simple screen menu system

    disableStepperDriver(); //switch off stepper motor power if option enabled
    manualControl(); //manual motor control to position stage before stack

    if (rbbuttonState == HIGH) { //use encoder to scroll through menu options

      lastEncoderPos = encoderPos; //store the current encoder position to compare with results of encoder_state()

      read_encoder(encoderPos); //check if the encoder has changed position and increment/decrement encoderPos accordingly

      encoderPos = constrain(encoderPos, 0, 8); //limits choice to specified range

      if (encoderPos == 8){ //when counter value exceeds number of menu items
        encoderPos = 1; //reset it to 1 again to create a looping navigation
      }

      if (encoderPos == 0){ //when counter value goes below minimum number of menu items
        encoderPos = 7; //reset it to 7 again to create a looping navigation
      }     
    } 

    if(lastEncoderPos != encoderPos){ //if the encoder position has changed
      updateScreen = true; //flag the screen as updatable
      //writing to the screen is relatively slow, so to keep the loop running quickly it should only be updated when the presented data has changed
    }

    switch (encoderPos) { //the menu options

      case 1: //this menu screen changes the number of increments to move each time

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = sliceSize; //store the current encoder position to compare with results of encoder_state()
        sliceSize = constrain(sliceSize, 1, 1000); //limits choice of input step size to specified range
        sliceSize += read_encoder(sliceSize);  //use encoder reading function to set value
        if(sliceSize != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged
        screen.clearScreen();
        screen.setFont(6);
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Set slice size:");
        screen.setPrintPos(6,4);
        frontLoadAndPrint(sliceSize); //frontload with correct number of zeroes and print to screen
        unitOfMeasure();
        printMenuArrows();
        updateScreen = false;

      }      

      break;

    case 2: //this menu screen changes the number of sliceNum to create in the stack

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = sliceNum; //store the current encoder position to compare with results of encoder_state()
        sliceNum = constrain(sliceNum, 1, 1000); //limits choice of input step size to specified range
        sliceNum += read_encoder(sliceNum);  //use encoder reading function to set value
        if(sliceNum != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Number of slices");
        screen.setPrintPos(6,4);
        frontLoadAndPrint(sliceNum); //then use that figure to frontload with correct number of zeroes and print to screen
        printMenuArrows();
        updateScreen = false;

      } 

      break;

    case 3: //this menu screen changes the number of seconds to wait for the camera to take a picture before moving again - 
      //you may want longer if using flashes for adequate recharge time or shorter with continuous lighting
      //to reduce overall time taken to complete the stack

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = pause; //store the current encoder position to compare with results of encoder_state()
        pause = constrain(pause, 1, 60); //limits choice of input step size to specified range
        pause += read_encoder(pause);  //use encoder reading function to set value
        if(pause != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Set pause time:");
        screen.setPrintPos(6,4);
        screen.print(pause); //divide millis by 1000 to display in seconds
        screen.print(" secs");
        printMenuArrows(); 
        updateScreen = false;

      }

      break;

    case 4: //toggles whether camera/subject is returned the starting position at the end of the stack

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = returnToStart; //store the current encoder position to compare with results of encoder_state()
        returnToStart = constrain(returnToStart, 1, 2); //limits choice of input step size to specified range
        returnToStart += read_encoder(returnToStart);  //use encoder reading function to set value
        if(pause != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Return to start:");
        screen.setPrintPos(3,4);
        if(returnToStart == 1){
          screen.print ("Enabled");
        }
        else {
          screen.print ("Disabled");
        }
        printMenuArrows();
        updateScreen = false;

      }

      break; 

    case 5: //this menu screen selects the unit of measure to use for sliceSize: Microns, Millimimeters or Centimeteres

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = measure; //store the current encoder position to compare with results of encoder_state()
        measure = constrain(measure, 1, 2); //limits choice of input step size to specified range
        measure += read_encoder(measure);  //use encoder reading function to set value
        if(measure != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Unit of measure:");
        screen.setPrintPos(6,4);
        unitOfMeasure();
        printMenuArrows();
        updateScreen = false;

      }

      break; 

    case 6: //this menu screen adjusts the stepper motor speed (delay in microseconds between sliceSize)
      //setting this to low may cause the motor to begin stalling or failing to move at all

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = stepDelay; //store the current encoder position to compare with results of encoder_state()
        stepDelay = constrain(stepDelay, 1000, 8000); //limits choice of input step size to specified range
        stepDelay += read_encoder(stepDelay);  //use encoder reading function to set value
        if(stepDelay != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Stepper speed: ");
        screen.setPrintPos(3,4);
        frontLoadAndPrint(stepDelay);
        screen.print (" uSecs");
        printMenuArrows();
        updateScreen = false;

      }

      break; 

    case 7: //this menu screen changes the number of images to take per focus slice (exposure bracketing support)

      if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
        lastEncoderPos = sliceBracket; //store the current encoder position to compare with results of encoder_state()
        sliceBracket = constrain(sliceBracket, 1, 10); //limits choice of input step size to specified range
        sliceBracket += read_encoder(sliceBracket);  //use encoder reading function to set value
        if(sliceBracket != lastEncoderPos){
          updateScreen = true;
        }
      }

      if (updateScreen){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        printHeader();
        screen.setPrintPos(0,2);
        screen.print("Set bracketing: ");
        screen.setPrintPos(2,4);
        frontLoadAndPrint(sliceBracket); //then use that figure to frontload with correct number of zeroes and print to screen
        printMenuArrows();           
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

        screen.clearScreen();
        printHeader(); 
      sliceCounter++; //count of pictures taken so far

      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    } 

    screen.setPrintPos(0,2);
    screen.print("Slice ");
    screen.print (sliceCounter);
    screen.print ("/");
    screen.print (sliceNum);
    screen.setPrintPos(0,4);
    screen.print("Advance ");
    screen.print (sliceSize);
    unitOfMeasure();

      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    } 

    enableStepperDriver();
    stepperDirection("forwards");
      ; //set the stepper direction for forward travel
      delay(100);

      for (int i = 0; i < sliceSize * 16 * measureMultiplier; i++){ ////adjust the number in this statement to tune distance travelled on your setup. In this case 16 is a product of 8x microstepping and a 2:1 gearing ratio
        stepSignal(); //send a step signal to the stepper driver

        if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 
    }

      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    } 
      disableStepperDriver(); //disable the stepper driver when not in use to save power  
      takePicture(); //send signal to camera to take picture

    }

    emptyTextRow(4);
    emptyTextRow(2);
    screen.print("Stack finished");
    delay(2000);

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //THE FOCUS STACK HAS FINISHED. IF THE OPTION IS ENABLED THE CAMERA STAGE IS RETURNED TO ITS START POSITION.
    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    if (returnToStart == 1){   
      stepperDirection("backwards"); //set the stepper direction for backward travel
      emptyTextRow(2);
      screen.print("Returning"); 
      int returnSteps = sliceSize * sliceCounter;
      emptyTextRow(4);
      screen.print (returnSteps);
      unitOfMeasure();

      enableStepperDriver();

      for (int i; i < returnSteps * 16 * measureMultiplier; i++){
        stepSignal();

        if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
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

  reading = digitalRead(pushButton);

  if (reading == LOW && previous == HIGH && millis() - time > debounce) {
    if (startStack == true) {
      startStack = false;
    } 
    else {
      startStack = true;
    }
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

  while (mcp.digitalRead(forwardControl) == LOW && inBounds()) {
    Serial.print("forwards");
    enableStepperDriver();
    stepperDirection("forwards");
    for (int i = 0; i<1; i++)
    {
      stepSignal(); //move forward
    }
    disableStepperDriver();
  }

  while (mcp.digitalRead(backwardControl) == LOW && inBounds()) {
    Serial.print("backwards");
    enableStepperDriver();
    stepperDirection("backwards");
    for (int i = 0; i<1; i++)
    {
      stepSignal(); //move backward
    }
    disableStepperDriver();
  }

}

/* CHECK IF CARRAIGE IS IN BOUNDS OF TRAVEL IE NEITHER LIMIT SWITCH IS TRIPPED */
boolean inBounds(){
  if (mcp.digitalRead(limitSwitchFront) == HIGH && mcp.digitalRead(limitSwitchBack == HIGH)){
    return true;
    } else {
      return false;
    }
  }

  /* SEND STEP SIGNAL TO EASYDRIVER TO TURN MOTOR */
  void stepSignal(){

  digitalWrite(doStep, LOW); //this LOW to HIGH change is what creates the
  digitalWrite(doStep, HIGH); //"Rising Edge" so the easydriver knows to when to step
  delayMicroseconds(stepDelay); //delay time between sliceSize, too fast and motor stalls

  if(!inBounds()){ //if a limit switch is triggered, pull back to within bounds and flag the collision
    clearLimitSwitch();
  }

}

/* SEND SIGNAL TO CAMERA TO TAKE PICTURE WITH DELAYS TO ALLOW SETTLING */
void takePicture(){
  for (int i = 1; i <= sliceBracket; i++){
    emptyTextRow(4); //wipe the last row of the screen ready for new content
    if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
    break;
  } 
    if(sliceBracket > 1){ //if more than one image is being taken, display the current sliceBracket number
      emptyTextRow(4); //wipe the last row of the screen ready for new content
      screen.print("Bracket ");
      screen.print(i);
      screen.print("/");
      screen.print(sliceBracket);
      delay(600);
    }
    emptyTextRow(4); //wipe the last row of the screen ready for new content
    screen.print("Pause for camera");

    digitalWrite(focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(shutter, HIGH); //trigger camera shutter

    delay(100); //small delay needed for camera to process above signals

    digitalWrite(shutter, LOW); //switch off camera trigger signal
    digitalWrite(focus, LOW); //switch off camera focus signal

    //A delay is required here for the camera to take an image, taking into account mirror-lockup and flash recharge time
    //The controller should still respond to a cancel event during this time so a non-blocking delay is required
    unsigned long initialMillis = millis(); //get the current millis
    unsigned long endMillis = initialMillis + (pause * 1000); //the time to finish the delay
    int countDown = 1; //slightly counter-intuitively counts upwards until equal to the pause time set

    while(endMillis > millis()){ //if the end of the pause time hasn't yet been reached
      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    }
      if(millis() > (initialMillis + (countDown * 1000)) && pause > countDown){ //if a second has elapsed since the last countdown statement, update it
        emptyTextRow(4); //wipe the last row of the screen ready for new content
        screen.print("Resume in ");
        screen.print(pause - countDown); //print number of seconds remaining
        screen.print("s");
        countDown++;
      }
    }
    countDown = 1; //reset the variable
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

/* SET STEPPER MOTOR DIRECTION */
void stepperDirection(String direction){
  if(direction == "backwards"){
    digitalWrite(dir, LOW);
  } 
  else { //if supplied string is anything other than "backwards" default to forwards
  digitalWrite(dir, HIGH);
}
}

/* PRINT SELECTED UNIT OF MEASURE TO SCREEN */
void unitOfMeasure() {        
  if (measure==1){
    measureMultiplier = 1;
    screen.print("mn");
  }
  if (measure==2){
    measureMultiplier = 1000;
    screen.print("mm");
  }
  if (measure==3){
    measureMultiplier = 10000;
    screen.print("cm");
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
  screen.print(menuvar);
}

/* READ THE ROTARY ENCODER
counts encoder pulses and registers only every nth pulse to adjust feedback sensitivity */
int read_encoder(int &encoderMenuVar){
  int8_t tmpdata = encoder_state(); 
  if(tmpdata){
    if(tmpdata == 1){
      encoderCount++;
    }

    if(encoderCount == 4){ //change this number to adjust encoder sensitivity
      encoderMenuVar++;
      encoderCount = 0;
    }

    if(tmpdata == -1){
      encoderCount--;
    }
    
    if(encoderCount == -4){ //change this number to adjust encoder sensitivity
      encoderMenuVar--;
      encoderCount = 0;
    }
  }
}

/* RETURN CHANGE IN ENCODER STATE */
int8_t encoder_state(){

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
  printHeader();
  screen.setPrintPos(0,2);
  screen.print("End of travel!");
  screen.setPrintPos(0,4);
  screen.print("Returning...");

  if (mcp.digitalRead(limitSwitchFront) == LOW){
    stepperDirection("backwards"); //reverse stepper motor direction
    while (mcp.digitalRead(limitSwitchFront) == LOW) //iterate doStep signal for as long as  the limit switch remains pressed 
    {  
      stepSignal();
    }
    stepperDirection("forwards"); //restore normal stepper motor direction
  }

  if (mcp.digitalRead(limitSwitchBack) == LOW){
    stepperDirection("forwards");
    ; //reverse stepper motor direction
    while (mcp.digitalRead(limitSwitchBack) == LOW) //iterate doStep signal for as long as  the limit switch remains pressed 
    {  
      stepSignal();
    }
  }

  screen.clearScreen();
  disableStepperDriver();

  if(startStack == true){ //if a stack was in progress, cancel it as there probably isn't space to continue
  startStack = false;
  stackCancelled();
}
}

/* PROCESS INTERRUPTS FROM THE MCP23017 */
void handleInterrupt(){

  //find out which pin on the mcp created the interrupt and its current state
  uint8_t pin=mcp.getLastInterruptPin();
  uint8_t val=mcp.getLastInterruptPinValue();

  detachInterrupt(0); //detach the interrupt while the current instance is dealt with

  //if a switchoff signal was recieved from the pushbutton controller pull kill pin low to power off
  if(pin == pbInterrupt && val == LOW) {
    mcp.digitalWrite(kill, LOW);
  }

  //if the rotary push button was pressed and a stack is not currently running, update its state
  if(pin == rotarypushButton && startStack == false){
    rotarybuttonChange();
  }

  attachInterrupt(0,handleInterrupt,FALLING); //re-attach the interrupt

}

/* RUN CLEANUP ON VARIABLES THEN GO BACK TO MENU SECTION */
void stackEnd(){

  encoderPos = 1; //set menu to first option screen
  sliceCounter = 0; //reset pic counter
  startStack = false; //return to menu options section
  updateScreen = true; //write the first menu item to the screen

}

/* CHECK IF THE MAIN BUTTON HAS BEEN PUSHED DURING STACK AND CANCEL IF YES */
boolean stackCancelled(){

  if(startStack == false){
    screen.clearScreen();
    printHeader();
    printPowerSource();
    screen.setPrintPos(0,2);
    screen.print("Stack cancelled");
    delay(1000);
    screen.clearScreen();
    return true;
  } 
  else {
    return false; 
  }

}

/* PRINT THE SCREEN HEADER */
void printHeader(){
  screen.clearScreen(); //clear the screen
  screen.drawBox(1,15,128,1); //draw a horizontal line to mark out a header section
  printPowerSource(); //display the current power source in top right corner
  screen.setPrintPos(0,0); //set text position in top left corner
  if(startStack == false){ //if the menu section is active, print the current menu item number
    screen.print("Menu ");
    screen.print(encoderPos);
    screen.print("/"); 
    screen.print("7");
  } 
  else { //or if a stack is in progress, print a message to that effect instead
    screen.print("Stacking");
  }
}

/* PRINT THE ACTIVE POWER SOURCE */
void printPowerSource(){
  if(mcp.digitalRead(stat) == LOW){
    //draw plug symbol (dc adapter connected)
    screen.drawBox(115,1,3,1);
    screen.drawBox(112,2,7,1);
    screen.drawBox(115,3,13,2);
    screen.drawBox(112,5,7,1);
    screen.drawBox(115,6,3,1);
  } 
  else {
    //draw battery symbol
    screen.drawBox(112,3,2,2);
    screen.drawBox(114,1,14,6);
  }
}

/* PRINT MENU NAVIGATION ARROWS
outward pointing arrows on either side of the menu
indicate turning the encoder will move to the next menu item */
void printMenuArrows(){
  if (rbbuttonState == HIGH) {
    //right outward arrow
    screen.drawBox(122,54,1,9);
    screen.drawBox(123,55,1,7);
    screen.drawBox(124,56,1,5);
    screen.drawBox(125,57,1,3);
    screen.drawBox(126,58,1,1);
    //left outward arrow
    screen.drawBox(5,54,1,9);
    screen.drawBox(4,55,1,7);
    screen.drawBox(3,56,1,5);
    screen.drawBox(2,57,1,3);
    screen.drawBox(1,58,1,1);
  } else {
    //right inward arrow
    screen.drawBox(122,58,1,1);
    screen.drawBox(123,57,1,3);
    screen.drawBox(124,56,1,5);
    screen.drawBox(125,55,1,7);
    screen.drawBox(126,54,1,9);
    //left inward arrow
    screen.drawBox(1,54,1,9);
    screen.drawBox(2,55,1,7);
    screen.drawBox(3,56,1,5);
    screen.drawBox(4,57,1,3);
    screen.drawBox(5,58,1,1);
  }
}

/* OVERWRITE SELECTED TEXT LINE WITH BLANKS AND RESET CURSOR TO BEGINNING OF LINE */
//avoids having to wipe the whole screen and re-output everything from scratch
//lineNum(Required) - the text row, between 1-5, to target
//charNum(Optional) - the character column, between 1-16, to start on
//endChar(Optional) - the character column, between 1-16, to end on 
void emptyTextRow(int lineNum) {
  screen.setPrintPos(0,lineNum);
  for(int i = 0; i <= 16; i++){
    screen.print(" ");//overwrite a single character slot with a blank string
  }
  screen.setPrintPos(0,lineNum);
}