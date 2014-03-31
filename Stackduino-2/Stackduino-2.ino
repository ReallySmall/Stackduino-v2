/*
 STACKDUINO
 
 A sketch to drive an Arduino based MacroPhotography Focus Stacking Controller
 http://www.flickr.com/photos/reallysmall/sets/72157632341602394/
 
 Key parts:
 ATMega 328
 Easydriver
 Bipolar stepper motor
 Rotary encoder with momentary push button
 16 x 2 HD44780 lcd
 2 x 4n35 optocoupler for camera connection
 
 Key resources used:
 Easydriver tutorial - http://danthompsonsblog.blogspot.com/2010/05/easydriver-42-tutorial.html
 Rotary encoder code - http://www.circuitsathome.com/mcu/reading-rotary-encoder-on-arduino
 */

#include <LiquidCrystal.h> //LCD library
#include <DigitalToggle.h> //digitalWrite toggle library

LiquidCrystal lcd(8, 9, 10, 11, 12, 13); //lcd pins

//rotary encoder pins
#define ENC_A A0
#define ENC_B A1
#define ENC_PORT PINC

//remaining digital pins
int pushButton = 2;  //start/ stop stack button
int rotarypushButton = 3; //select/ unselect menu item button
int dir = 4; //stepper motor direction
int doStep = 5; //send step signal to stepper motor
int focus = 6; //send an autofocus signal to the camera
int shutter = 7; //send a shutter signal to the camera
int forwardControl = A2; //for manual positioning
int enable = A3; //enable and disable easydriver when not stepping to reduce heat and power consumption (future functionality)
int limitSwitches = A4; //limit switches to stop stepping if end of travel on rail is reached at either end
int backwardControl = A5; //for manual positioning

//defaults
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
int lcdloopCounter = 0; //count number of loops to periodically update lcd
int encoderCounter = 0; //count pulses from encoder
int varSize = 0; //used for frontloading menu item numbers with 0's
boolean disableEasydriver = true; //whether to disable easydriver betweem steps to save power and heat
boolean reverseFwdBwd = false; //change to true to reverse direction of the forward and backward manual control buttons
boolean returnToStart = false; //whether camera/ subject is returned to starting position at end of stack

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

  lcd.begin(16, 2);
  Serial.begin(9600);  

  attachInterrupt(0, buttonChange, CHANGE);  //pushButton on interrupt 0
  attachInterrupt(1, rotarybuttonChange, CHANGE);  //rotarypushButton on interrupt 1

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
  pinMode(limitSwitches, INPUT);

  digitalWrite(focus, LOW);
  digitalWrite(shutter, LOW);
  digitalWrite(ENC_A, HIGH);
  digitalWrite(ENC_B, HIGH);
  digitalWrite(pushButton, HIGH);
  digitalWrite(rotarypushButton, HIGH);
  digitalWrite(forwardControl, HIGH);
  digitalWrite(backwardControl, HIGH);
  digitalWrite(enable, LOW);
  digitalWrite(limitSwitches, HIGH);

  lcd.setCursor(0, 0);
  lcd.print("Stackduino");
  delay(1000);
  lcd.clear();

}

void loop(){

  if (digitalRead(limitSwitches) == LOW){ //if controller is started up hitting a limit switch disable main functions and print warning to lcd
    disableEasyDriver(); //disable easydr

    lcdloopCounter++;

    if (lcdloopCounter >= 40){ //periodically refresh lcd display

      lcd.setCursor(0, 0);
      lcd.print("End of travel!  ");
      lcd.setCursor(0, 1);
      lcd.print("                ");
      lcdloopCounter = 0;

    }      

    manualControl(); //Provide manual motor control to move away from limit switch and resolve error

  }

  else{ //if limit switches HIGH run all functions normally

    if (buttonState == HIGH){ //this section allows manual control and configures settings using a simple lcd menu system
      
      disableEasyDriver(); //switch off stepper motor power if option enabled
      manualControl(); //manual motor control to position stage before stack

      if (rbbuttonState == HIGH) { //use encoder to scroll through menu of settings

        int8_t tmpdata = read_encoder(); //counts encoder pulses and registers only every nth pulse to adjust feedback sensitivity
        if(tmpdata){
          if(tmpdata == 1){
            encoderCounter++;
          }

          if(encoderCounter == 2){ //change this number to adjust encoder sensitivity
            rotaryCounter++;
            encoderCounter = 0;
          }

          if(tmpdata == -1){
            encoderCounter--;
          }
          if(encoderCounter == -2){ //change this number to adjust encoder sensitivity
            rotaryCounter--;
            encoderCounter = 0;
          }
        }

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

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          steps = constrain(steps, 1, 1000); //limits choice of input step size to specified range
          steps += read_encoder ();  //use encoder reading function to set value of steps variable
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){ //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Set step size:  ");
          lcd.setCursor(0, 1);
          varSize = steps; //transfer the value of the menu variable to varSize
          frontLoadAndPrint(); //then use that figure to frontload with correct number of zeroes and print to screen
          lcd.print (steps  , DEC);
          unitOfMeasure();
          lcd.print("         "); //fill rest of display line with empty chars to overwrite conetnt of previous screen  
          lcdloopCounter = 0;

        }      

        break;

      case 2: //this menu screen changes the number of pictures to take in the stack

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          numPictures = constrain(numPictures, 10, 5000); //limits choice of number of pictures to take to specified range
          numPictures += (read_encoder () * 10); //use encoder reading function to set value of numPictures variable - 
          //changes in increments of 10 for quick selection of large numbers
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){  //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Set num steps:  ");
          lcd.setCursor(0, 1);
          varSize = numPictures; //transfer the value of the menu variable to varSize
          frontLoadAndPrint(); //then use that figure to frontload with correct number of zeroes and print to screen
          lcd.print (numPictures, DEC);
          lcd.print ("        ");
          lcdloopCounter = 0;

        } 

        break;

      case 3: //this menu screen changes the number of seconds to wait for the camera to take a picture before moving again - 
        //you may want longer if using flashes for adequate recharge time or shorter with continuous lighting
        //to reduce overall time taken to complete the stack

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          setPause = constrain(setPause, 1000, 30000); //limits choice of pause time between pictures to specified range
          setPause += (read_encoder () * 1000); //use encoder reading function to set value of setPause variable
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){  //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Set pause time: ");
          lcd.setCursor(0, 1);
          if (setPause < 10000){
            lcd.print (0, DEC); //adds one leading zero to triple digit setPause numbers on the display
          }
          lcd.print ((setPause / 1000), DEC); //divide millis by 1000 to display in seconds
          lcd.print(" seconds      ");  
          lcdloopCounter = 0;

        }

        break;

      case 4: //toggles whether camera/subject is returned the starting position at the end of the stack

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          returnToStart = constrain(returnToStart, true, false); //limits choice of returnToStart to specified range
          returnToStart += read_encoder (); //use encoder reading function to set value of returnToStart variable
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){  //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Return to start:");
          lcd.setCursor(0, 1);
          if(returnToStart == true){
            lcd.print ("Enabled         ");
          }
          else {
            lcd.print ("Disabled        ");
          }
          lcdloopCounter = 0;

        }

        break; 

      case 5: //this menu screen selects the unit of measure to use for steps: Microns, Millimimeters or Centimeteres

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          unitofMeasure = constrain(unitofMeasure, 1, 3); //limits choice of returnToStart to specified range
          unitofMeasure += read_encoder (); //use encoder reading function to set value of returnToStart variable
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){  //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Unit of measure:");
          lcd.setCursor(0, 1);

          switch (unitofMeasure){ 

          case 1:

            lcd.print ("Microns (mn)    ");
            uomMultiplier = 1;
            break;

          case 2:

            lcd.print ("Millimetres (mm)");
            uomMultiplier = 1000;
            break;

          case 3: 

            lcd.print ("Centimetres (cm)");
            uomMultiplier = 10000;
            break;
          }

          lcdloopCounter = 0;

        }

        break; 

      case 6: //this menu screen adjusts the stepper motor speed (delay in microseconds between steps)
        //setting this to low may cause the motor to begin stalling or failing to move at all

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          stepSpeed = constrain(stepSpeed, 1000, 9000); //limits choice of returnToStart to specified range
          stepSpeed += (read_encoder () * 1000); //use encoder reading function to set value of stepSpeed variable
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){  //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Stepper speed:  ");
          lcd.setCursor(0, 1);
          lcd.print (stepSpeed, DEC);
          lcd.print (" microsecs  ");
          lcdloopCounter = 0;

        }

        break; 

      case 7: //this menu screen changes the number of images to take per focus slice (exposure bracketing support)

        if (rbbuttonState == LOW) { //press rotary encoder button within this menu item to edit variable
          bracket = constrain(bracket, 1, 10); //limits choice of input step size to specified range
          bracket += read_encoder ();  //use encoder reading function to set value of steps variable
        }

        lcdloopCounter++;

        if (lcdloopCounter >= 40){  //the screen hardware is slow so only print to it every 40th loop to minimise input lag

          lcd.setCursor(0, 0);
          lcd.print("Set bracketing: ");
          lcd.setCursor(0, 1);
          varSize = bracket; //transfer the value of the menu variable to varSize
          frontLoadAndPrint(); //then use that figure to frontload with correct number of zeroes and print to screen
          lcd.print (bracket  , DEC);          
          lcd.print("            "); //fill rest of display line with empty chars to overwrite conetnt of previous screen  
          lcdloopCounter = 0;

        }      

        break;

      }
    } //end of setup menu section

    else { //this section runs the stack when the pushButton is pressed

      disableEasyDriver();

      for (int i = 0; i < numPictures; i++){ //loop the following actions for number of times dictated by var numPictures

        picCounter++; //count of pictures taken so far

        lcd.clear();
        lcd.print("Moving ");
        lcd.print (steps);
        unitOfMeasure();
        lcd.setCursor(0, 1);
        lcd.print("Step ");
        lcd.print (picCounter);
        lcd.print (" of ");
        lcd.print (numPictures);

        delay(500);
        enableEasyDriver();
        digitalWrite(dir, HIGH); //set the stepper direction for backward travel (if your motor is wired the other way around you may need to reverse this)
        delay(100);

        int i = 0; //counter for motor steps
        while (i < steps * 16 * uomMultiplier && digitalRead(limitSwitches) == HIGH){ //adjust the number in this statement to tune distance travelled on your setup. In this case 16 is a product of 8x microstepping and a 2:1 gearing ratio
          stepSignal();
          i++;
        }
        i = 0; //reset counter
        disableEasyDriver();

        if (digitalRead(limitSwitches) == LOW){ //stop motor and reverse if limit switch hit
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
      lcd.setCursor(0, 0);
      lcd.print("Stack finished");
      delay(2000);
      lcd.clear(); 
      if (returnToStart == 1){   
        digitalWrite(dir, LOW); //set the stepper direction for backward travel (if your motor is wired the other way around this you may need to reverse this)
        delay(100);
        lcd.setCursor(0, 0);
        lcd.print("<< Returning..."); 
        int returnSteps = steps * picCounter;
        lcd.setCursor(0, 1);
        lcd.print (returnSteps);
        unitOfMeasure();

        enableEasyDriver();

        int i = 0; //counter for motor steps
        while (i < returnSteps * 16 * uomMultiplier && digitalRead(limitSwitches) == HIGH){

          stepSignal();
          i++;
        }
        i = 0; //reset counter

        disableEasyDriver();

        if (digitalRead(limitSwitches) == LOW){ //stop motor and reverse if limit switch hit
          retreat();
        }  

        lcd.clear();
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

/* PULL CARRIAGE BACK FROM TRIPPED LIMIT SWITCH */
void retreat(){
  enableEasyDriver();
  digitalToggle(dir); //reverse motor direction to move away from limitswitch
  lcd.setCursor(0, 0);
  lcd.print("End of travel!  ");
  lcd.setCursor(0, 1);
  lcd.print("Reversing...    ");

  while (digitalRead(limitSwitches) == LOW) //iterate doStep signal for as long as eitherlimit switch remains pressed 
  {  
    stepSignal();
  }

  digitalToggle(dir); //reset motor back to original direction once limit switch is no longer pressed
  lcd.clear();
  disableEasyDriver();
}

/* MOVE CARRIAGE BACKWARD AND FORWARD USING PUSH BUTTONS */
void manualControl(){

  while (digitalRead(forwardControl) == LOW) {
    enableEasyDriver();
    int i;
    if(reverseFwdBwd == false){
      digitalWrite(dir, HIGH); 
    } 
    else {
      digitalWrite(dir, LOW);
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
    if(reverseFwdBwd == false){
      digitalWrite(dir, LOW); 
    } 
    else {
      digitalWrite(dir, HIGH);
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
  if(bracket == 1) { //If bracketing is not enabled
    lcd.clear();
    lcd.print("Pause for image");
    lcd.setCursor(0, 1);
    lcd.print("(");
    lcd.print ((setPause / 1000), DEC);
    lcd.print(" seconds)");

    digitalWrite(focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(shutter, HIGH); //trigger camera shutter

    delay(200); //small delay needed for camera to process above signals

    digitalWrite(shutter, LOW); //switch off camera trigger signal
    digitalWrite(focus, LOW); //switch off camera focus signal

    delay(setPause); //pause to allow for camera to take picture with mirror lockup and to allow flashes to recharge before next shot

    lcd.clear();
  } 
  else {  //If bracketing is enabled
    int bracketNum = 1; //counter for brackets
    for (int i = 0; i < bracket; i++)
    {

      lcd.clear();
      lcd.print("Bracketed image:");
      lcd.setCursor(0, 1);
      lcd.print(bracketNum);
      lcd.print(" of ");
      lcd.print(bracket);
      delay(1000);
      bracketNum++;
      lcd.clear();
      lcd.print("Pause for image");
      lcd.setCursor(0, 1);
      lcd.print("(");
      lcd.print ((setPause / 1000), DEC);
      lcd.print(" seconds)");

      digitalWrite(focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
      digitalWrite(shutter, HIGH); //trigger camera shutter

      delay(200); //small delay needed for camera to process above signals

      digitalWrite(shutter, LOW); //switch off camera trigger signal
      digitalWrite(focus, LOW); //switch off camera focus signal

      delay(setPause); //pause to allow for camera to take picture with mirror lockup and to allow flashes to recharge before next shot

      lcd.clear();
    }
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
    lcd.print(" mn");
  }
  if (unitofMeasure==2){
    lcd.print(" mm");
  }
  if (unitofMeasure==3){
    lcd.print(" cm");
  }
}

/* FRONT PAD MENU ITEM NUMBERS WITH ZEROES */

void frontLoadAndPrint() {
  if (varSize < 10){
    lcd.print (000, DEC); //adds three leading zeros to single digit Step size numbers on the display
  }
  if (varSize < 100){
    lcd.print (00, DEC); //adds two leading zeros to double digit Step size numbers on the display
  }
  if (varSize < 1000){
    lcd.print (0, DEC); //adds one leading zero to triple digit Step size numbers on the display
  }
}

