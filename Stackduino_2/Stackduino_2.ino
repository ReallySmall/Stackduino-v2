/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STACKDUINO 2                                                                                        //       
//  An Arduino compatible MacroPhotography Focus Stacking Controller optimised for mobile use           //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DEPENDENCIES                                                                                        //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

#define _Digole_Serial_SPI_ //OLED screen - must be configured with solder jumper to run in SPI mode
#include "DigoleSerial.h" //https://github.com/chouckz/HVACX10Arduino/blob/master/1.0.1_libraries/DigoleSerial/DigoleSerial.h
#include "Wire.h"
#include "Adafruit_MCP23017.h" //customised version with interrupts by husio-org: https://github.com/adafruit/Adafruit-MCP23017-Arduino-Library/pull/6

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  INSTANTIATE OBJECTS                                                                                 //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

Adafruit_MCP23017 mcp; //16 pin port expander
DigoleSerialDisp screen(8,9,10);  //OLED screen - SPI mode - Pin 8: data, 9:clock, 10: SS

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DEFINE REMAINING ATMEGA PINS                                                                        //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

#define ENC_A A0 //rotary encoder
#define ENC_B A1 //rotary encoder
#define ENC_PORT PINC //rotary encoder

const byte mcp_interrupt = 2; //MCP23017 interrupt
const byte button_main = 3;  //start/ stop stack button
const byte stepper_driver_direction = 4; //direction pin on A4988 stepper driver
const byte stepper_driver_do_step = 5; //step pin on A4988 stepper driver
const byte camera_focus = 6; //camera autofocus
const byte camera_shutter = 7; //camera shutter
const byte batt_fet = 16; //connect/ disconnect battery sample voltage line
const byte batt_sense = 17; //analogue read battery sample voltage line
const byte ext_analogue_1 = A6; //analogue pin broken out through DB15 - unused (for future functionality)
const byte ext_analogue_2 = A7; //analogue pin broken out through DB15 - unused (for future functionality)

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DEFINE MCP23017 PINS                                                                                //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

const byte stepper_driver_forward = 0; //for manual positioning
const byte stepper_driver_backward = 1; //for manual positioning
const byte power_in_status = 2; //LTC4412 stat pin - indicates whether controller is currently powered by wall adapter or battery
const byte switch_off_flag = 3; //LTC2950 interrupt pin - signals to the controller that it is about to be switched off
const byte switch_off = 4; //LTC2950 kill pin - disables 5v regulator and switches off controller
const byte limit_switch_front = 5; //limit switch to stop stepper motor if end of travel is reached
const byte limit_switch_back = 6; //limit switch to stop stepper motor if end of travel is reached
const byte bluetooth_toggle = 7; //toggles power to bluetooth breakout board (if attached)
const byte ext_digital_1 = 8; //digital pin broken out through DB15 - unused (for future functionality)
const byte ext_digital_2 = 9;//digital pin broken out through DB15 - unused (for future functionality)
const byte button_rotary = 10; //select/ unselect menu item button
const byte stepper_driver_ms1 = 11; //toggles A4988 stepper driver MS1 pin to control degree of microstepping
const byte stepper_driver_ms2 = 12; //toggles A4988 stepper driver MS2 pin to control degree of microstepping
const byte stepper_driver_ms3 = 13; //toggles A4988 stepper driver MS3 pin to control degree of microstepping
const byte stepper_driver_enable = 14; //disable/enables A4988 stepper driver to conserve power
const byte stepper_driver_sleep = 15; //sleep/wake A4988 stepper driver to conserve power

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SETTINGS                                                                                            //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

int bluetooth_enabled = 0; //toggle power to bluetooth port
int camera_bracket = 2; //number of images to bracket per focus slice
int camera_pause = 5; //default time in seconds to wait for camera to take picture and allow any flashes to recharge
int encoder_pulse_counter = 0; //counts pulses from encoder - used by encoderUpdate to adjust responsiveness
int loop_counter = 0; //counts how many loops have elapsed - used to execute housekeeping functions periodically
int menu_item = 1; //which menu item to display when turning rotary encoder
int mirror_lockup = 0; //set to 1 to enable camera mirror lockup
int raw_voltage_reading = 0; //raw anologue reading of current battery voltage
int resistor_1 = 10000; // battery monitor voltage divider R1
int resistor_2 = 3300; // battery monitor voltage divider R2
int return_to_start = 0; //whether linear stage is returned to starting position at end of stack
int slices = 10; //default number of focus slices to make in the stack
int slice_size = 10; //default depth of each focus slice - used with unit_of_measure
int slice_counter = 0; //count of number of focus slices made so far in the stack - used by returnToStart()
int step_delay = 2000; //delay in microseconds between stepper motor steps, governing motor speed - too low may cause missed steps or stalling
int tuning = 16; //multiplier used to tune distance travelled - e.g. adjust this so that 1 motor step = 1 micron on your setup
int unit_of_measure = 1; //whether slice_size should be  microns, mm or cm
int unit_of_measure_multiplier = 1; //multiplier for active unit_of_measure (1, 1000 or 10,000)
float calculated_voltage_reading = 0; //the actual battery voltage as calculated through the divider
float voltage_denominator = (float)resistor_2 / (resistor_1 + resistor_2);
boolean bluetooth_connected = false;
boolean start_stack = false;
boolean stepper_driver_disable = true; //whether to disable the A4988 stepper driver when possible to save power and heat
boolean screen_update = true; //true whenever encoder or button functions are called, prompting a screen update
volatile boolean mcp_interrupt_fired = false; //true whenever a pin on the mcp23017 with a listeniner went low

//main push button toggle
volatile int button_main_state = HIGH; //the current state of the output pin
volatile int button_main_reading; //the current reading from the input pin
volatile int button_main_previous = LOW; //the previous reading from the input pin
volatile long button_main_time = 0; //the last time the output pin was toggled
volatile long button_main_debounce = 400; //the debounce time, increase if the output flickers

//rotary push button toggle
volatile int button_rotary_state = HIGH; //the current state of the output pin
volatile int button_rotary_reading; //the current reading from the input pin
volatile int button_rotary_previous = LOW; //the previous reading from the input pin
volatile long button_rotary_time = 0; //the last time the output pin was toggled
volatile long button_rotary_debounce = 400; //the debounce time, increase if the output flickers

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SETUP                                                                                               //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SET ATMEGA PINMODES AND PULLUPS                                                                     //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

pinMode(mcp_interrupt, INPUT);
digitalWrite(mcp_interrupt, HIGH);
pinMode(button_main, INPUT);
digitalWrite(button_main, HIGH); 
pinMode(ENC_A, INPUT);
digitalWrite(ENC_A, HIGH); 
pinMode(ENC_B, INPUT);
digitalWrite(ENC_B, HIGH); 
pinMode(stepper_driver_direction, OUTPUT); 
digitalWrite(stepper_driver_direction, LOW); 
pinMode(stepper_driver_do_step, OUTPUT); 
digitalWrite(stepper_driver_do_step, LOW);     
pinMode(camera_focus, OUTPUT); 
digitalWrite(camera_focus, LOW);
pinMode(camera_shutter, OUTPUT); 
digitalWrite(camera_shutter, LOW);
pinMode(batt_fet, OUTPUT); 
digitalWrite(batt_fet, LOW);

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  START MCP23017 AND SET PINMODES AND PULLUPS                                                         //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  mcp.begin();// using default address 0

  int mcp_input_pins[] = {0,1,2,3,4,5,6,10};
  for(int i=0; i < 8; i++){
    mcp.pinMode(mcp_input_pins[i], INPUT);
    mcp.pullUp(mcp_input_pins[i], HIGH);
  }

  int mcp_output_pins[] = {7,8,9,11,12,13,14,15};
  for(int i=0; i < 8; i++){
    mcp.pinMode(mcp_output_pins[i], OUTPUT);
    mcp.digitalWrite(mcp_output_pins[i], HIGH);
  }
  
  mcp.digitalWrite(8, LOW); //the currently unused pin needs setting LOW
  mcp.digitalWrite(9, LOW); //the currently unused pin needs setting LOW
  mcp.digitalWrite(14, LOW); //the a4988 enable pin needs setting LOW

  //set up interrupts
  mcp.setupInterrupts(true,false,LOW);
  mcp.setupInterruptPin(stepper_driver_forward, FALLING);
  mcp.setupInterruptPin(stepper_driver_backward, FALLING);
  mcp.setupInterruptPin(limit_switch_front,FALLING);
  mcp.setupInterruptPin(limit_switch_back,FALLING);
  mcp.setupInterruptPin(switch_off_flag,FALLING);
  mcp.setupInterruptPin(button_rotary,FALLING); 
  mcp.setupInterruptPin(power_in_status,FALLING);  

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  START SERIAL CONNECTION AND ATTACH INTERRUPTS                                                       //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  Serial.begin(19200); //start serial

  //screen.displayConfig(0); //disable config message on the Digole OLED screen
  //screen.displayStartScreen(0); //disable splash on the Digole OLED screen

  attachInterrupt(0, mcpInterrupt, FALLING); //mcp23017 on interrupt 0
  attachInterrupt(1, buttonMainToggle, FALLING); //main push button on interrupt 1

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CHECK LIMIT SWITCHES                                                                                //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  //stepperDriverClearLimitSwitch(); //make sure neither limit switch is hit on startup - rectify if so

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BATTERY VOLTAGE READING                                                                             //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void batteryMonitor(){ /* CONNECT THE BATTERY VOLTAGE SAMPLE LINE AND READ THE VOLTAGE THROUGH A DIVIDER */
  if(power_in_status == HIGH){ //if currently running on battery
    //digitalWrite(batt_fet, HIGH); //connect the battery voltage line for reading
    //raw_voltage_reading = digitalRead(batt_sense); //get an analogue reading of the battery voltage through the divider
    //calculated_voltage_reading = ((batt_sense / 1024) * 3.3) / voltage_denominator; //calculate the actual voltage
    //digitalWrite(batt_fet, LOW); //disconnect the battery voltage line again
    screen_update = true; //flag the screen as updatable to display newest battery voltage reading
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BLUETOOTH                                                                                           //
//  requires a compatible bluetooth breakout board (e.g. HC-05) connected to header H_1                 //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void bluetooth_power(){
  Serial.print("bluetooth");
  if(bluetooth_enabled){ //enable power to VCC pin on header H_1
    mcp.digitalWrite(bluetooth_toggle, LOW);
    } else { //cut power to VCC pin on header H_1
      mcp.digitalWrite(bluetooth_toggle, HIGH);
    }
  }

  void bluetooth_communication(){
    if(bluetooth_enabled){
    //TODO
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BUTTON FUNCTIONS                                                                                    //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

void buttonMainToggle(){ /* RETURN CURRENT TOGGLE STATE OF MAIN PUSH BUTTON */

  button_main_reading = digitalRead(button_main);

  if (button_main_reading == LOW && button_main_previous == HIGH && millis() - button_main_time > button_main_debounce) {
    if(start_stack == false){
      start_stack = true; 
      } else {
        start_stack = false; 
      }
      button_main_time = millis();    
    }

    button_main_previous = button_main_reading;

  } 

void buttonRotaryToggle(){ /* RETURN CURRENT TOGGLE STATE OF ROTARY ENCODER'S PUSH BUTTON */

  button_rotary_reading = mcp.digitalRead(button_rotary);
  if (button_rotary_reading == LOW && button_rotary_previous == HIGH && millis() - button_rotary_time > button_rotary_debounce) {
    if (button_rotary_state == HIGH){
      button_rotary_state = LOW;
      } else {
        button_rotary_state = HIGH;
      }
      button_rotary_time = millis();    
    }

    button_rotary_previous = button_rotary_reading;

  } 

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CAMERA INTERFACE FUNCTIONS                                                                          //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

void cameraProcessImages(){ /* SEND SIGNAL TO CAMERA TO TAKE PICTURE(S) */
  Serial.print("Take picture");
  for (int i = 1; i <= camera_bracket; i++){
    screenEmptyTextRow(4); //wipe the last row of the screen ready for new content
    if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes end this loop early (ie cancel the stack), print a message, then return to the menu section
    break;
  } 
    if(camera_bracket > 1){ //if more than one image is being taken, display the current position in the bracket
      screenEmptyTextRow(4); //wipe the last row of the screen ready for new content
      screen.print("Bracket ");
      screen.print(i);
      screen.print("/");
      screen.print(camera_bracket);
      delay(600);
    }
    screenEmptyTextRow(4); //wipe the last row of the screen ready for new content
    screen.print("Pause for camera");

    cameraShutterSignal(); //take the image

    //A delay is required here for the camera to take an image, taking into account any flash recharge time
    //The controller should still respond to a cancel event during this pause though so a non-blocking delay is required
    unsigned long initial_millis = millis(); //get the current millis
    unsigned long end_millis = initial_millis + (camera_pause * 1000); //when the pause ends
    int count_down = 1; //slightly counter-intuitively counts upwards until equal to the pause time set

    while(end_millis > millis()){ //if the end of the pause time hasn't yet been reached
      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    }
      if(millis() > (initial_millis + (count_down * 1000)) && camera_pause > count_down){ //if a second has elapsed since the last countdown statement, update it
        screenEmptyTextRow(4); //wipe the last row of the screen ready for new content
        screen.print("Resume in ");
        screen.print(camera_pause - count_down); //print number of seconds remaining
        screen.print("s");
        count_down++;
      }
    }
    count_down = 1; //reset the counter for next time
  }

}

void cameraShutterSignal() { /* SEND SHUTTER SIGNAL */
//if mirror lockup is enabled, send 2 delay seperated signals to the camera to first lift the mirror, then open the shutter 

for (int i = 0; i < mirror_lockup + 1; i++)
{

  if(mirror_lockup && i == 0){
    screen.clearScreen();
    screen.print("Mirror up");
  }
Serial.print("Send shutter signal");
    digitalWrite(camera_focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(camera_shutter, HIGH); //trigger camera shutter

    delay(500); //small delay needed for camera to process above signals

    digitalWrite(camera_shutter, LOW); //switch off camera trigger signal
    digitalWrite(camera_focus, LOW); //switch off camera focus signal

    if(mirror_lockup && i == 0){
      delay(2000); //sets the delay between mirror up and shutter
      screen.clearScreen();
    }
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ROTARY ENCODER FUNCTIONS                                                                            //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/  

int8_t encoderRead(){ /* RETURN CHANGE IN ENCODER STATE */

  static int8_t enc_states[] = { 
    0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 //use this line instead to increment encoder in the opposite direction
    //0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0
  };
  static uint8_t old_AB = 0;
  /**/
  old_AB <<= 2; //remember previous state
  old_AB |= ( ENC_PORT & 0x03 ); //add current state
  return ( enc_states[( 0x0f & old_AB )]);

}

void encoderUpdate(int &variable, int lower, int upper, int multiplier = 1){ /* CHANGE THE ACTIVE MENU VARIABLE'S VALUE USING THE ENCODER */
  
  variable = constrain(variable, lower, upper); //keep variable value within specified range  
  
  int8_t encoder_data = encoderRead(); //counts encoder pulses and registers only every nth pulse to adjust feedback sensitivity
  
  if(encoder_data){
    if(encoder_data == 1){
      encoder_pulse_counter++;
    }

    if(encoder_pulse_counter == 2){ //change this number to adjust encoder sensitivity 4 or 2 are usually the values to use
      variable = variable + multiplier;
      Serial.print(variable);
      screen_update = true; //as a variable has just been changed by the encoder, flag the screen as updatable
      encoder_pulse_counter = 0;
    }

    if(encoder_data == -1){
      encoder_pulse_counter--;
    }
    if(encoder_pulse_counter == -2){ //change this number to adjust encoder sensitivity 4 or 2 are usually the values to use
      variable = variable - multiplier;
      Serial.print(variable);
      screen_update = true; //as a variable has just been changed by the encoder, flag the screen as updatable
      encoder_pulse_counter = 0;
    }
  }

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MCP 23017 INTERRUPT FUNCTIONS                                                                       //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void mcpInterrupt(){ /* PROCESS INTERRUPTS FROM THE MCP23017 */

  mcp_interrupt_fired = true;

}

void handleMcpInterrupt(){

  //find out which pin on the mcp created the interrupt and its current state
  uint8_t pin = mcp.getLastInterruptPin();
  uint8_t val = mcp.getLastInterruptPinValue();

  detachInterrupt(0); //detach the interrupt while the current instance is dealt with

  //if a switchoff signal was recieved from the pushbutton controller, pull kill pin low to power off
  if(pin == switch_off_flag && val == LOW) {
    mcp.digitalWrite(switch_off, LOW);
  }
  
  //if a maual stage control button was pressed
  if((pin == stepper_driver_forward && val == LOW || pin == stepper_driver_backward && val == LOW) && start_stack == false){
    stepperDriverManualControl();
  }

  //if the rotary push button was pressed and a stack is not currently running, update its state
  if(pin == button_rotary && val == LOW && start_stack == false){
    //buttonRotaryToggle();
    Serial.print("buttom");
  }

  //if a limit switch was hit, clear it
  if(pin == limit_switch_front && val == LOW || pin == limit_switch_back && val == LOW){
    stepperDriverClearLimitSwitch();
  }

  //if the power source changed, update the screen
  if(pin == power_in_status){
    screen_update = true;
  }

  mcp_interrupt_fired = false;
  attachInterrupt(0, mcpInterrupt, FALLING); //re-attach the interrupt

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DISPLAY FORMATTING FUNCTIONS                                                                        //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void screenEmptyTextRow(int line_num) { /* OVERWRITE SELECTED TEXT LINE WITH BLANKS AND RESET CURSOR TO BEGINNING OF LINE */
//avoids having to wipe the whole screen and re-output everything from scratch
screen.setPrintPos(0, line_num);
for(int i = 0; i <= 16; i++){
    screen.print(" ");//overwrite a single character slot with a blank string
  }
  screen.setPrintPos(0, line_num);
}

void screenFrontLoadAndPrint(int menu_var) { /* FRONT PAD MENU ITEM VARIABLE NUMBERS WITH ZEROES */
//maintains consistent formatting
if (menu_var < 10){
    screen.print (000, DEC); //adds three leading zeros to single digit Step size numbers on the display
  }
  if (menu_var < 100){
    screen.print (00, DEC); //adds two leading zeros to double digit Step size numbers on the display
  }
  if (menu_var < 1000){
    screen.print (0, DEC); //adds one leading zero to triple digit Step size numbers on the display
  }
  screen.print(menu_var);
}

void screenPrintHeader(){ /* PRINT THE HEADER */
  screen.clearScreen(); //clear the screen
  screen.drawBox(1,15,128,1); //draw a horizontal line to mark out a header section
  screenPrintBluetooth(); //display an icon for current bluetooth connection state
  screenPrintPowerSource(); //display the current power source in top right corner
  screen.setPrintPos(0,0); //set text position in top left corner
  if(start_stack == false){ //if the menu section is active, print the current menu item number
    screen.print("Menu");
  } 
  else { //or if a stack is in progress, print a message to that effect instead
    screen.print("Stack");
  }
}

void screenPrintMenuArrows(){ /* PRINT MENU NAVIGATION ARROWS */
  if (button_rotary_state == HIGH) {
    //outward pointing arrows on either side of the menu indicate turning the encoder will move to the next menu item 
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
    //inward pointing arrows on either side of the menu indicate turning the encoder will change the value of the current menu item 
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

void screenPrintBluetooth(){ /* PRINT BLUETOOTH ICONS */
  if(bluetooth_enabled && !bluetooth_connected){ //if the bluetooth header (H_1) is active but there isn't an active pairing
    //TODO
  }
  if(bluetooth_enabled && bluetooth_connected){ //if the bluetooth header (H_1) is active and there's an active pairing
    //TODO
  }
}

void screenPrintPowerSource(){ /* PRINT THE ACTIVE POWER SOURCE */
  if(mcp.digitalRead(power_in_status) == LOW){
    //draw plug symbol (dc adapter connected)
    screen.drawBox(115,1,3,1);
    screen.drawBox(112,2,7,1);
    screen.drawBox(115,3,13,2);
    screen.drawBox(112,5,7,1);
    screen.drawBox(115,6,3,1);
  } 
  else {
    //draw battery symbol
    screen.setPrintPos(7,0);
    screen.print(calculated_voltage_reading);
    screen.print("v");
    //Serial.print(calculated_voltage_reading);
    screen.drawBox(112,3,2,2);
    screen.drawBox(114,1,14,6);
  }
}

void screenUnitOfMeasure() { /* PRINT SELECTED UNIT OF MEASURE TO SCREEN */        
  if (unit_of_measure == 1){
    unit_of_measure_multiplier = 1;
    screen.print("mn");
  }
  if (unit_of_measure == 2){
    unit_of_measure_multiplier = 1000;
    screen.print("mm");
  }
  if (unit_of_measure == 3){
    unit_of_measure_multiplier = 10000;
    screen.print("cm");
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  END OF STACK FUNCTIONS                                                                              //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

boolean stackCancelled(){ /* CHECK IF THE MAIN BUTTON HAS BEEN PUSHED DURING STACK AND END STACK IF YES */

  if(start_stack == false){
    screen.clearScreen();
    screenPrintHeader();
    screenPrintPowerSource();
    screen.setPrintPos(0,2);
    screen.print("Stack cancelled");
    Serial.print("Stack cancelled");
    delay(1000);
    screen.clearScreen();
    return true;
  } 
  else {
    return false; 
  }

}

void stackEnd(){ /* RUN CLEANUP THEN GO BACK TO MENU SECTION */

  screenEmptyTextRow(4);
  screenEmptyTextRow(2);
  screen.print("Stack finished");
  delay(2000);

  if (return_to_start){   
      stepperDriverDirection("backwards"); //set the stepper direction for backward travel
      
      int return_steps = slice_size * slice_counter;
      
      screenEmptyTextRow(2);
      screen.print("Returning");
      screenEmptyTextRow(4);
      screen.print (return_steps);
      screenUnitOfMeasure();
      
      stepperDriverEnable();

      for (int i; i < return_steps * tuning * unit_of_measure_multiplier; i++){

        stepperDriverStep();

        if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 
    }

    stepperDriverDisable(); 
    screen.clearScreen();
  }

  menu_item = 1; //set menu to first option screen
  slice_counter = 0; //reset pic counter
  start_stack = false; //return to menu options section
  screen_update = true; //write the first menu item to the screen
  Serial.print("Stack end");

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STEPPER MOTOR CONTROL FUNCTIONS                                                                     //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void stepperDriverClearLimitSwitch(){ /* PULL LINEAR STAGE AWAY FROM TRIPPED LIMIT SWITCH */
  stepperDriverEnable();
  screen.clearScreen();
  screenPrintHeader();
  screen.setPrintPos(0,2);
  screen.print("End of travel!");
  screen.setPrintPos(0,4);
  screen.print("Returning...");

  if (mcp.digitalRead(limit_switch_front) == LOW){
    Serial.print("Limit switch front hit");
    stepperDriverDirection("backwards"); //reverse stepper motor direction
    while (mcp.digitalRead(limit_switch_front) == LOW) //turn stepper motor for as long as  the limit switch remains pressed 
    {  
      stepperDriverStep();
    }
    stepperDriverDirection("forwards"); //restore normal stepper motor direction
  }

  if (mcp.digitalRead(limit_switch_back) == LOW){
    Serial.print("Limit switch back hit");
    stepperDriverDirection("forwards");
    ; //reverse stepper motor direction
    while (mcp.digitalRead(limit_switch_back) == LOW) //turn stepper motor for as long as  the limit switch remains pressed 
    {  
      stepperDriverStep();
    }
  }

  screen.clearScreen();
  stepperDriverDisable();

  if(start_stack == true){ //if a stack was in progress, cancel it as there probably isn't space to continue
    start_stack = false;
    stackCancelled();
  }
}

void stepperDriverEnable() { /* ENABLE THE STEPPER DRIVER */
  mcp.digitalWrite(stepper_driver_enable, LOW);
}

void stepperDriverDisable() { /* DISABLE THE STEPPER DRIVER WHEN NOT IN USE (IF OPTION IS SET) */
  if(stepper_driver_disable) {
    mcp.digitalWrite(stepper_driver_enable, HIGH);
  } 
  else {
    mcp.digitalWrite(stepper_driver_enable, LOW);
  } 
}

void stepperDriverDirection(String direction){ /* SET STEPPER MOTOR DIRECTION */
  if(direction == "backwards"){

    digitalWrite(stepper_driver_direction, LOW);
  } 
  else { //if supplied string is anything other than "backwards" default to forwards

  digitalWrite(stepper_driver_direction, HIGH);
}
}

boolean stepperDriverInBounds(){ /* CHECK IF LINEAR STAGE IS IN BOUNDS OF TRAVEL I.E. NEITHER LIMIT SWITCH IS TRIPPED */
  if (mcp.digitalRead(limit_switch_front) == HIGH && mcp.digitalRead(limit_switch_back == HIGH)){
    return true;
  } else {
    return false;
  }
}

void stepperDriverManualControl(){ /* MOVE STAGE BACKWARD AND FORWARD USING PUSH BUTTONS */

    if (mcp.digitalRead(stepper_driver_forward) == LOW && stepperDriverInBounds()) {
      screen.clearScreen();
      screenPrintHeader();
      screen.setPrintPos(0,2);
      screen.print("Moving forwards");
      Serial.print("Going forwards");
      screen.setPrintPos(0,4);
      screen.print(">-->>-->>-->>-->");
      while (mcp.digitalRead(stepper_driver_forward) == LOW && stepperDriverInBounds()) {
        stepperDriverEnable();
        stepperDriverDirection("forwards");
        for (int i = 0; i<1; i++) {
        stepperDriverStep(); //move forwards
      }
      stepperDriverDisable();
    }
    screen_update = true; //go back to displaying the active menu item once manual control button is no longer pressed
  }

  if (mcp.digitalRead(stepper_driver_backward) == LOW && stepperDriverInBounds()) {
    screen.clearScreen();
    screenPrintHeader();
    screen.setPrintPos(0,2);
    screen.print("Moving backwards");
    Serial.print("Going backwards");
    screen.setPrintPos(0,4);
    screen.print("<--<<--<<--<<--<");
    while (mcp.digitalRead(stepper_driver_backward) == LOW && stepperDriverInBounds()) {
      stepperDriverEnable();
      stepperDriverDirection("backwards");
      for (int i = 0; i<1; i++) {
        stepperDriverStep(); //move backwards
      }
      stepperDriverDisable();
    }
    screen_update = true; //go back to displaying the active menu item once manual control button is no longer pressed
  }

}

void stepperDriverStep(){ /* SEND STEP SIGNAL TO A4988 STEPPER DRIVER */

  digitalWrite(stepper_driver_do_step, LOW); //this LOW to HIGH change is what creates the
  digitalWrite(stepper_driver_do_step, HIGH); //"Rising Edge" so the easydriver knows to when to step
  delayMicroseconds(step_delay); //delay time between sliceSize, too fast and motor stalls

  if(!stepperDriverInBounds()){ //if a limit switch is triggered, pull back to within bounds and flag the collision
    stepperDriverClearLimitSwitch();
  }

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP                                                                                           //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void loop(){

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MENU SCREENS AND MANUAL STAGE CONTROL                                                               //                                                                                                     
//                                                                                                      //
//  Manual control of linear stage is only enabled when in menu section - prevents running stacks       //
//  being ruined by accidental button presses                                                           //
//                                                                                                      //
//  Menu navigation controlled by a rotary encoder with integral push button. Toggle state of push      // 
//  button defines whether turning the enocoder displays the next menu item or changes the value of     //
//  the displayed menu item.                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  if (start_stack == false){ //this section allows manual control and configures settings using a simple screen menu system

    loop_counter++;
    

    if(loop_counter > 5000){ //run periodical housekeeping tasks
      batteryMonitor(); //measure the battery voltage if running on battery power
      loop_counter = 0;
      Serial.print("menu");
    }
    
    if(mcp_interrupt_fired){ //if an interrupt fired from the mcp23017, find out which pin fired it and deal with it
      handleMcpInterrupt();
    }

    bluetooth_communication();

    stepperDriverDisable(); //switch off stepper motor power if option set

    if (button_rotary_state == HIGH) { //use encoder to scroll through menu items

      encoderUpdate(menu_item, 0, 10); //display the currently selected menu item

      if (menu_item == 10){ //when counter value exceeds number of menu items
        menu_item = 1; //reset it to 1 again to create a looping navigation
      }

      if (menu_item == 0){ //when counter value goes below minimum number of menu items
        menu_item = 9; //reset it to 8 again to create a looping navigation
      }  

    } 

    switch (menu_item) { //the menu options

      case 1: //this menu item changes the number of increments to move each time

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(slice_size, 1, 1000);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Set slice size:");
        screenPrintMenuArrows();
        screen.setPrintPos(5,4);
        screenFrontLoadAndPrint(slice_size);
        screenUnitOfMeasure();
        screen_update = false;
      }    

      break;

    case 2: //this menu item changes the number of slices to create in the stack

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(slices, 10, 5000, 10);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Number of slices");
        screen.setPrintPos(6,4);
        screenFrontLoadAndPrint(slices);
        screenPrintMenuArrows();
        screen_update = false;

      } 

      break;

    case 3: //this menu item changes the number of seconds to wait for the camera to take a picture before moving again - 
      //you may want longer if using flashes for adequate recharge time or shorter with continuous lighting
      //to reduce overall time taken to complete the stack

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(camera_pause, 1, 60);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Set pause time:");
        screen.setPrintPos(6,4);
        screen.print(camera_pause); //divide millis by 1000 to display in seconds
        screen.print("secs");
        screenPrintMenuArrows(); 
        screen_update = false;

      }

      break;

    case 4: //this menu item toggles whether camera/subject is returned the starting position at the end of the stack

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(return_to_start, 0, 1);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Return to start:");
        screen.setPrintPos(5,4);
        if(return_to_start){
          screen.print ("Enabled");
        }
        else {
          screen.print ("Disabled");
        }
        screenPrintMenuArrows();
        screen_update = false;

      }

      break; 

    case 5: //this menu items selects the unit of measure to use for sliceSize: Microns, Millimimeters or Centimeteres

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(unit_of_measure, 1, 3);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Unit of measure:");
        screen.setPrintPos(7,4);
        screenUnitOfMeasure();
        screenPrintMenuArrows();
        screen_update = false;

      }

      break; 

    case 6: //this menu item adjusts the stepper motor speed (delay in microseconds between sliceSize)
      //setting this too low may cause the motor to begin stalling or failing to move at all

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(step_delay, 1000, 8000);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Stepper speed: ");
        screen.setPrintPos(3,4);
        screenFrontLoadAndPrint(step_delay);
        screen.print (" uSecs");
        screenPrintMenuArrows();
        screen_update = false;

      }

      break; 

    case 7: //this menu item changes the number of images to take per focus slice (exposure bracketing)

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(camera_bracket, 1, 10);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Set bracketing: ");
        screen.setPrintPos(6,4);
        screenFrontLoadAndPrint(camera_bracket);
        screenPrintMenuArrows();           
        screen_update = false;

      }      

      break;

    case 8: //this menu item screen toggles mirror lockup for the camera

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(mirror_lockup, 0, 1);
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Mirror Lockup: ");
        screen.setPrintPos(4,4);
        if(mirror_lockup){
          screen.print("Enabled");
          } else {
            screen.print("Disabled");
          }
          screenPrintMenuArrows();           
          screen_update = false;

        }      

        break;

    case 9: //this menu item screen toggles power to an external bluetooth board e.g. HC-05

      if (button_rotary_state == LOW) { //press rotary encoder button within this menu item to edit variable
        encoderUpdate(bluetooth_enabled, 0, 1);
        bluetooth_power();
      }

      if (screen_update){ //only write to the screen when a change to the variables has been flagged

        screen.clearScreen();
        screenPrintHeader();
        screen.setPrintPos(0,2);
        screen.print("Bluetooth port:");
        screen.setPrintPos(4,4);
        if(bluetooth_enabled){
          if(bluetooth_connected){
            screen.print("Connected");  
            } else {
              screen.print("Unconnected");  
            }
            } else {
              screen.print("Disabled");
            }
            screenPrintMenuArrows();           
            screen_update = false;

          }      

          break;

        }
  } //end of setup menu section
  else { 

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE FOCUS STACK.                                                                                    //
//  STARTS WHEN THE MAIN PUSH BUTTON IS PRESSED, CANCELLED IF PRESSED AGAIN                             // 
//  THE LOOP ADVANCES THE STAGE, TAKES A PICTURE AND REPEATS.                                           //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

bluetooth_communication();
Serial.print("Stack started");
    stepperDriverDisable(); //disable the stepper driver when not in use to save power

    for (int i = 0; i < slices; i++){ //for each focus slice in the stack...

      screen.clearScreen();
      screenPrintHeader(); 
      slice_counter++; //count slices made in the stack so far

      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 

      //print the current position in the stack to the screen
      screen.setPrintPos(0,2);
      screen.print("Slice ");
      screen.print (slice_counter);
      screen.print ("/");
      screen.print (slices);
      screen.setPrintPos(0,4);
      screen.print("Advance ");
      screen.print (slice_size);
      screenUnitOfMeasure();

      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 

      stepperDriverEnable(); //enable the A4988 stepper driver
      stepperDriverDirection("forwards"); //set the stepper direction for forward travel
      delay(100);

      for (int i = 0; i < slice_size * tuning * unit_of_measure_multiplier; i++){ 

        stepperDriverStep(); //send a step signal to the stepper driver

        if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
        break;
      } 

    }

      if(stackCancelled()){ //check the button to cancel the stack hasn't been pressed - if yes we'll end this loop early (ie cancel the stack), print a message, then return to the menu section
      break;
    } 

      stepperDriverDisable(); //disable the stepper driver when not in use to save power  
      
      cameraProcessImages(); //take image(s)

    }

    stackEnd();

  } 
}

