/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STACKDUINO 2 (BETA)                                                                                       //
//                                                                                                      //
//  https://github.com/ReallySmall/Stackduino-2                                                         //
//                                                                                                      //
//  An Arduino compatible Focus Stacking Controller optimised for mobile use                            //
//  Richard Iles 2014                                                                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DEFINES AND DEPENDENCIES                                                                            //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

#define _Digole_Serial_SPI_ //OLED screen - must be configured with solder jumper to run in SPI mode
#define ENC_A A0 //rotary encoder
#define ENC_B A1 //rotary encoder
#define ENC_PORT PINC //rotary encoder

#include "DigoleSerial.h" //https://github.com/chouckz/HVACX10Arduino/blob/master/1.0.1_libraries/DigoleSerial/DigoleSerial.h
#include "Wire.h"
#include "Adafruit_MCP23017.h"
#include "LowPower.h" //https://github.com/rocketscream/Low-Power

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  INSTANTIATE REQUIRED LIBRARY OBJECTS                                                                //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

Adafruit_MCP23017 mcp; //16 pin port expander
DigoleSerialDisp screen(8,9,10);  //OLED screen - SPI mode - Pin 8: data, 9:clock, 10: SS

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
// SET UP REMAINING ATMEGA PINS                                                                         //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

const byte mcp_interrupt = 2; //MCP23017 interrupt
const byte button_main = 3;  //start/ stop stack button
const byte stepper_driver_direction = 4; //direction pin on A4988 stepper driver
const byte stepper_driver_do_step = 5; //step pin on A4988 stepper driver
const byte camera_focus = 6; //camera autofocus
const byte camera_shutter = 7; //camera shutter
const byte batt_sense = A2; //analogue read battery sample voltage line
const byte batt_fet = 17; //connect/ disconnect battery sample voltage line
const byte ext_analogue_1 = A6; //analogue pin broken out through DB15 - unused (for future functionality)
const byte ext_analogue_2 = A7; //analogue pin broken out through DB15 - unused (for future functionality)

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SET UP MCP23017 PINS                                                                                //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

const byte stepper_driver_forward = 0; //for manual positioning
const byte stepper_driver_backward = 1; //for manual positioning
const byte power_in_status = 2; //LTC4412 stat pin - indicates whether controller is currently powered by wall adapter or battery
const byte switch_off_flag = 3; //LTC2950 interrupt pin - signals to the controller that it is about to be switched off
const byte switch_off = 4; //LTC2950 kill pin - disables 3.3v regulator and switches off controller
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

//the controller relies on the assumption that 1 step signal sent by the motor driver will move the 
//linear stage by 1 (or as close as possible to 1) increment of the selected unit of measure for travel.
//to achieve this relationship, the number of step signals generated should be multiplied by a constant 
//(obtained through a little maths and experimentation) which is unique to your hardware.
//as you may use Stackduino on more than one linear stage (e.g. studio and field setups), which each require
//unique constants, these are stored in an array:
const float hardware_calibration_settings[] = {1, 1}; //hardware calibration settings
int active_hardware_calibration = 0; //array index of selected hardware calibration setting

//screen menu strings
const char* menu_strings[] = {"System settings", 
                              "Set slice size:", 
                              "Number of slices", 
                              "Set pause time:", 
                              "Mirror Lockup:", 
                              "Set bracketing:", 
                              "Return to start:", 
                              "Unit of measure:", 
                              "Stepper speed:", 
                              "Microstepping", 
                              "Linear Stage:", 
                              "Bluetooth:"};
                              
const char* menu_strings_uom[] = {"",
                                  "mn",
                                  "mm",
                                  "cm"};

int bluetooth_enabled = 1; //toggle power to bluetooth port
boolean bluetooth_connected = false; //whether bluetooth is connected to a device

int camera_bracket = 1; //number of images to bracket per focus slice
int camera_pause = 5; //default time in seconds to wait for camera to take picture and allow any flashes to recharge
int mirror_lockup = 0; //set to 1 to enable camera mirror lockup, 0 to disable

volatile boolean start_stack = false; //false when in the menu, true when stacking
volatile boolean traverse_menus = true; //true when scrolling through menus, false when changing a menu item's value
volatile boolean mcp_interrupt_fired = false; //true whenever a pin on the mcp23017 with a listener went low
int menu_item = 1; //the active menu item
int setting_changed = 0; //count increments the active menu setting should be changed by on the next poll
boolean publish_update = true; //true whenever functions are called which change a setting's value

int incoming_serial; //store latest byte from incoming serial stream
float calculated_voltage_reading = 0; //the actual battery voltage as calculated through the divider
int system_timeout_sleep = 10; //minutes of inactivity to wait until controller enters sleep mode - set to 0 to disable
int system_timeout_off = 20; //minutes of inactivity to wait until controller switches itself off - set to 0 to disable
int minutes_elapsed = 0; //number of 1 minute periods elapsed
boolean system_idle = true; //true until the controller is interacted with

boolean stepper_driver_disable = true; //whether to disable the A4988 stepper driver when possible to save power and heat
int step_delay = 2000; //delay in microseconds between stepper motor steps, governing motor speed - too low may cause missed steps or stalling
int available_micro_stepping[] = {1, 2, 4, 8, 16}; //degrees of microstepping available for the a4988 stepper driver should use (1 is full stepping)
int active_micro_stepping = 4; //array index of selected micro-stepping setting

int return_to_start = 0; //whether linear stage is returned to its starting position at the end of stack
int slices = 10; //default number of focus slices to make in the stack
int slice_size = 10; //default depth of each focus slice - used with unit_of_measure
int slice_counter = 0; //count of number of focus slices made so far in the stack
int unit_of_measure_multipliers[] = {1, 1000, 10000}; //multiplier for active unit of measure
int active_unit_of_measure_multiplier = 1; //array index of selected multiplier

//push buttons
volatile int button_main_reading, button_rotary_reading; //the current reading from the input pin
volatile int button_main_previous = LOW, button_rotary_previous = LOW; //the previous reading from the input pin
volatile long button_main_time = 0, button_rotary_time = 0; //the last time the output pin was toggled

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SETUP                                                                                               //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void setup() {

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SET ATMEGA PINMODES AND PULLUPS                                                                     //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  pinMode(mcp_interrupt, INPUT_PULLUP);
  pinMode(button_main, INPUT_PULLUP); 
  pinMode(ENC_A, INPUT_PULLUP);  
  pinMode(ENC_B, INPUT_PULLUP);
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

  int mcp_input_pins[] = {0,1,2,3,5,6,10};
  
  for(int i=0; i < 7; i++){

    mcp.pinMode(mcp_input_pins[i], INPUT);
    mcp.pullUp(mcp_input_pins[i], HIGH);

  }

  int mcp_output_pins[] = {4,7,8,9,11,12,13,14,15};
  
  for(int i=0; i < 9; i++){

    mcp.pinMode(mcp_output_pins[i], OUTPUT);
    mcp.digitalWrite(mcp_output_pins[i], HIGH);

  }
  
  mcp.digitalWrite(8, LOW); //the currently unused pin needs setting LOW
  mcp.digitalWrite(9, LOW); //the currently unused pin needs setting LOW
  mcp.digitalWrite(14, LOW); //the a4988 enable pin needs setting LOW

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SET UP INTERRUPTS                                                                                   //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  attachInterrupt(0, mcpInterrupt, FALLING); //ATMega external interrupt 0
  attachInterrupt(1, buttonMainToggle, FALLING); //ATMega external interrupt 1

  pciSetup(ENC_A); //ATMega pin change interrupt
  pciSetup(ENC_B); //ATMegapin change interrupt 
  
  //MCP23017 interrupts - feed to ATMega external interrupt 0
  mcp.setupInterrupts(true, false, LOW);
  mcp.setupInterruptPin(stepper_driver_forward, FALLING);
  mcp.setupInterruptPin(stepper_driver_backward, FALLING);
  mcp.setupInterruptPin(switch_off_flag,FALLING);
  mcp.setupInterruptPin(power_in_status,FALLING);
  mcp.setupInterruptPin(button_rotary,FALLING);   

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  FINAL PRE-START CHECKS AND SETUP                                                                    //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

  Serial.begin(19200); //start serial
  stepperDriverMicroStepping(available_micro_stepping[active_micro_stepping]); //set the degree of microstepping used by the a4988
  stepperDriverClearLimitSwitch(); //make sure neither limit switch is hit on startup - rectify if so
  batteryMonitor(); //check the battery level
  bluetoothTogglePower(); //switch on Bluetooth if the option is enabled by default
  
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  SYSTEM FUNCTIONS                                                                                    //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void systemSleep(boolean staged_shutdown = true){ 
/* SEND TO SLEEP
 *
 * uses the lowPower libary
 *
 * staged_shutdown => when true controller sleeps for 4 seconds, counting up additional minutes until finally
 *                    switching the controller off. when false the controller sleeps indefinitely until 
 *                    woken by an interrupt (any button).
 */
  
  staged_shutdown == true ? LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF) : LowPower.powerDown(SLEEP_FOREVER, ADC_OFF, BOD_OFF); 

}

void systemWake(){ 
/* RESTART ANY SUSPENDED PERIPHERALS 
 *
 * tasks for the controller to do once it wakes up
 */

  screen.backLightOn();//switch off screen
  system_idle = true;
  minutes_elapsed = 0;

}

void systemOff(){ 
/* POWER OFF 
 *
 * controller powers itself off by pulling the enable pin on the voltage regulator low
 */

  Serial.print("system off");
  //mcp.digitalWrite(switch_off, LOW);

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  UPDATE THE SCREEN IF DATA CHANGES                                                                   //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void screenUpdate(int print_pos_y = 2, int print_pos_x = 0, String text = ""){ /* WIPE THE SCREEN, PRINT THE HEADER AND SET THE CURSOR POSITION */

  screen.clearScreen(); //clear the screen
  screen.setPrintPos(0,0); //set text position in top left corner
  start_stack == true ? screen.print("Stack") : screen.print("Menu"); //display whether menu or stack is active
  screen.drawBox(1,15,128,1); //draw a horizontal line to mark out a header section
  screenPrintBluetooth(); //display an icon for current bluetooth connection state
  screenPrintPowerSource(); //display the current power source in top right corner
  screenPrintMenuArrows(); //print menu arrows (if a stack is not running)
  screen.setPrintPos(print_pos_x, print_pos_y); //set text position to beginning of content area
  if(text != ""){
    screen.print(text); //print any supplied text string
  }
  system_idle = false; //flag that the system was recently used

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BATTERY VOLTAGE READING                                                                             //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void batteryMonitor(){ /* CONNECT THE BATTERY VOLTAGE SAMPLE LINE AND READ THE VOLTAGE THROUGH A DIVIDER */
  
  if(mcp.digitalRead(power_in_status) == HIGH){ //if currently running on battery
  
    static int resistor_1 = 10000; // battery monitor voltage divider R1
    static int resistor_2 = 3300; // battery monitor voltage divider R2
    static float last_calculated_voltage_reading = 0.00; // the last battery reading
    static float raw_voltage_reading = 0; //raw anologue reading of current battery voltage
    static float voltage_denominator = (float)resistor_2 / (resistor_1 + resistor_2);

    digitalWrite(batt_fet, HIGH); //connect the battery voltage line for reading
    raw_voltage_reading = analogRead(batt_sense); //get an analogue reading of the battery voltage through the divider
    calculated_voltage_reading = ((raw_voltage_reading / 1023) * 3.3) / voltage_denominator; //calculate the actual voltage
    digitalWrite(batt_fet, LOW); //disconnect the battery voltage line again
    
    if(calculated_voltage_reading != last_calculated_voltage_reading){
      publish_update = true; //flag the screen as updatable to display newest battery voltage reading
    } 
    
    last_calculated_voltage_reading = calculated_voltage_reading;
    
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BLUETOOTH                                                                                           //
//  requires a compatible bluetooth breakout board (e.g. HC-05) connected to header H_1                 //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void bluetoothTogglePower(){ /* TOGGLES VCC TO THE BLUETOOTH HEADER */
  
  bluetooth_enabled == true ? mcp.digitalWrite(bluetooth_toggle, LOW) : mcp.digitalWrite(bluetooth_toggle, HIGH);

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  BUTTONS                                                                                             //
//  toggle a boolean state with a debounced button press                                                 //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

void buttonMainToggle(){ /* RETURN CURRENT TOGGLE STATE OF MAIN PUSH BUTTON */

  system_idle = false;
  static int button_debounce = 400;
  button_main_reading = digitalRead(button_main);

  if (button_main_reading == LOW && button_main_previous == HIGH && millis() - button_main_time > button_debounce) {
    start_stack = start_stack == true ? false : true; 
    button_main_time = millis();    
  }

  button_main_previous = button_main_reading;

} 

void buttonRotaryToggle(){ /* RETURN CURRENT TOGGLE STATE OF ROTARY ENCODER'S PUSH BUTTON */

  static int button_debounce = 400;
  button_rotary_reading = mcp.digitalRead(button_rotary);
  
  if (button_rotary_reading == LOW && button_rotary_previous == HIGH && millis() - button_rotary_time > button_debounce) {
    traverse_menus = traverse_menus == true ? false : true;
    button_rotary_time = millis();    
  }
    
  button_rotary_previous = button_rotary_reading;

} 

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CAMERA INTERFACE FUNCTIONS                                                                          //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

void cameraProcessImages(){ /* SEND SIGNAL TO CAMERA TO TAKE PICTURE(S) */
  
  for (int i = 1; i <= camera_bracket; i++){

    pause(1000); //allow vibrations to settle

    if(camera_bracket > 1){ //if more than one image is being taken, display the current position in the bracket
      screenPrintProgress();
      screen.setPrintPos(0,4);
      screen.print("Bracket ");
      screen.print(i);
      screen.print("/");
      screen.print(camera_bracket);
    }

    screenPrintProgress();
    screen.setPrintPos(0,4);
    screen.print("Pause for camera");

    cameraShutterSignal(); //take the image

    for(int i = 1; i <= camera_pause; i++){
      
      pause(1000);
      screenPrintProgress();
      screen.setPrintPos(0,4);
      screen.print("Resume in ");
      screen.print(camera_pause - i); //print number of seconds remaining
      screen.print("s");

      if(stackCancelled()){ //exit early if the stack has been cancelled
        break; 
      }

    }

    if(stackCancelled()){ //exit early if the stack has been cancelled
      break; 
    }

  }
}

void cameraShutterSignal() { /* SEND SHUTTER SIGNAL */
//if mirror lockup is enabled, send 2 delay seperated signals to the camera to first lift the mirror, then open the shutter 

  for (int i = 0; i <= mirror_lockup; i++){

    if(mirror_lockup && i == 0){
      screenUpdate(4);
      screen.print("Mirror up");
    }
      
    digitalWrite(camera_focus, HIGH); //trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(camera_shutter, HIGH); //trigger camera shutter

    pause(500); //small delay needed for camera to process above signals

    digitalWrite(camera_shutter, LOW); //switch off camera trigger signal
    digitalWrite(camera_focus, LOW); //switch off camera focus signal

    if(mirror_lockup && i == 0){
      pause(2000); //sets the delay between mirror up and shutter
      screenUpdate();
    }

    if(stackCancelled()){ //exit early if the stack has been cancelled
      break; 
    }

  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  TASKS WHICH RUN EVERY SO OFTEN ON A TIMER                                                           //
//  Periodically check the battery level                                                                //
//  Sleep or switch off the controller if it has been unused for n minutes                              //                                       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/  

void cronJobs(){

  static unsigned long a_minute = millis() + 60000; // 1 minute from now

  if(millis() >= a_minute){ //every 1 minute
    Serial.println("a minute");
    batteryMonitor(); //check the battery level
    
    if(!system_idle){ //if the controller has been used in the past minute
      minutes_elapsed = 0; //restart the minutes counter
      system_idle = true; //and reset the system flag
    } else {
      minutes_elapsed++; //keep count of minutes elapsed
       Serial.println(minutes_elapsed);
    }
    
    a_minute = millis() + 60000; //reset the timer again to 1 minute from now

  }

  //if enabled, after n minutes put the idle controller in sleep mode
  if(system_timeout_sleep && (system_timeout_sleep < system_timeout_off) && (minutes_elapsed >= system_timeout_sleep)){ 
    
    byte naps = 0;

    while(system_idle){
      
      systemSleep(); //have a nap for 4 seconds
      naps++;
     Serial.println(naps); 
      
      if(naps == 15){ 
        minutes_elapsed++;
        naps = 0;
      }

      if(system_timeout_off && (minutes_elapsed >= system_timeout_off)){
        //systemOff();
        Serial.print("off");
      }

    }

    systemWake(); //an interrupt woke the controller

  }

  //if enabled (and sleep mode is disabled) after n minutes switch off the controller 
  if(system_timeout_off > system_timeout_sleep && (minutes_elapsed >= system_timeout_off)){
    //systemOff();
    Serial.print("off-2");
  }

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  ROTARY ENCODER FUNCTIONS                                                                            //
//  Using work by                                                                                       //
//  http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros           //  
////////////////////////////////////////////////////////////////////////////////////////////////////////*/  

int8_t encoderRead(){ /* RETURN CHANGE IN ENCODER STATE */

  static int8_t enc_states[] = { 
    //0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 //use this line instead to increment encoder in the opposite direction
    0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0 
  };
  
  static uint8_t old_AB = 0;

  old_AB <<= 2; //remember previous state
  old_AB |= ( ENC_PORT & 0x03 ); //add current state
  
  int8_t encoder_data = ( enc_states[( 0x0f & old_AB )]);

  static int8_t encoder_pulse_counter = 0; //counts pulses from encoder

  if(encoder_data){ //if Gray Code returns a valid input (1 or -1) 
    encoder_data == 1 ? encoder_pulse_counter++ : encoder_pulse_counter--; 
  }

  if(encoder_pulse_counter > 3){ //record the encoder was moved by one click (detent)
    setting_changed++;
    encoder_pulse_counter = 0;
  } 
  
  else if(encoder_pulse_counter < -3) { //record the encoder was moved by one click (detent) in the opposite direction
    setting_changed--;
    encoder_pulse_counter = 0;
  }

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  INTERRUPT FUNCTIONS                                                                                 //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void pciSetup(byte pin)
{
    *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
    PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
    PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}

ISR (PCINT1_vect){ //read the encoder if the analogue pin port is interrupted
  
  encoderRead();

}  

void mcpInterrupt(){ /* PROCESS INTERRUPTS FROM THE MCP23017 */

  mcp_interrupt_fired = true;
  system_idle = false;

}

void handleMcpInterrupt(){
  
  detachInterrupt(0); //detach the interrupt while the current instance is dealt with

  //find out which pin on the mcp created the interrupt and its current state
  uint8_t pin = mcp.getLastInterruptPin();
  uint8_t val = mcp.getLastInterruptPinValue();

  //if a switchoff signal was recieved from the pushbutton controller, pull kill pin low to power off
  if(pin == switch_off_flag && val == LOW) {
    systemOff();
  }
  
  //if a manual stage control button was pressed
  if(((pin == stepper_driver_forward) || (pin == stepper_driver_backward)) && start_stack == false){
    stepperDriverManualControl();
  }

  //if a limit switch was hit, clear it
  if(pin == limit_switch_front && val == LOW || pin == limit_switch_back && val == LOW){
    stepperDriverClearLimitSwitch();
  }

  //if the power source changed, update the screen
  if(pin == power_in_status){
    publish_update = true;
  }

  //if the rotary encoder button was pressed, check the current state
  if(pin == button_rotary){
    buttonRotaryToggle();
  }
  
  EIFR=0x01;
  mcp_interrupt_fired = false;
  attachInterrupt(0, mcpInterrupt, FALLING); //re-attach the interrupt

}


/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  NON BLOCKING DELAY                                                                                  //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void pause(int length){ //delays are needed throughout the stacking process, but these pauses should be non-blocking

    unsigned long initial_millis = millis(); //get the current millis
    unsigned long end_millis = initial_millis + length; //when the pause ends

    while(end_millis > millis()){ //if the end of the pause time hasn't yet been reached

      if(stackCancelled()){ //exit early if the stack has been cancelled
        break; 
      }

    }
    
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  DIGOLE OLED DISPLAY FORMATTING FUNCTIONS                                                            //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void screenPrintMenuArrows(){ /* PRINT MENU NAVIGATION ARROWS */

  if(start_stack == false) {
    if (traverse_menus == true) {
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

}

void screenPrintBluetooth(){ /* PRINT BLUETOOTH ICONS */

  if(bluetooth_enabled && !bluetooth_connected){ //if the bluetooth header (H_1) is active but there isn't an active pairing
    //TODO PRINT AN INACTIVE ICON
  }

  if(bluetooth_enabled && bluetooth_connected){ //if the bluetooth header (H_1) is active and there's an active pairing
    //TODO PRINT AN ACTIVE ICON.
  }

}

void screenPrintCentre(String text, int print_pos_y = 4, byte text_colour = 0){ /* PRINT TEXT CENTERED ON SCREEN */

  int text_length = text.length();
  int offset = (16 - text_length) / 2; //calculate the offset for the number of characters in the string

  screen.setPrintPos(offset, print_pos_y);

  if(text_length % 2 != 0){ //dividing an odd numbered int by 2 discards the remainder, creating an offset which is half a text character width too short
    screen.setTextPosOffset(4, 0); //so the text needs to be nudged to the right a bit on a pixel level to centre it properly 
  } 

  //if(!traverse_menus){
    //screen.drawBox(16,48,96,16);
    //screen.setColor(text_colour); //default to black text on white as this is normally printing a selected menu item
  //}
  screen.print(text); //finally, print the centered text to the screen

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
    screen.drawBox(112,3,2,2);
    screen.drawBox(114,1,14,6);
  }

}

void screenPrintProgress(){

  screenUpdate(); 
  screen.print("Slice ");
  screen.print (slice_counter);
  screen.print ("/");
  screen.print (slices);

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CONTROL VIA SERIAL                                                                                  //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void serialCommunications(){

  if (Serial.available() > 0) {

    system_idle = false;  
    int incoming_serial = Serial.read(); // read the incoming byte:

    switch(incoming_serial){

      case 'a': //start/ stop stack
        start_stack = start_stack == true ? false : true;
        break;

      case 'b': //toggle rotary encoder button state
        traverse_menus = traverse_menus == true ? false : true;
        break;

      case 'c': //increment active user setting
        setting_changed++;
        break;

      case 'd': //decrement active user setting
        setting_changed--;
        break;

      case 'e': //forward
        stepperDriverManualControl();
        break;

      case 'f': //backward
        stepperDriverManualControl();
        break;

      case 'g': //take test shot
        cameraShutterSignal();
        break;

      case 'h': //toggle bluetooth
        bluetoothTogglePower();
        break;

      case 'i': //put controller into sleep mode
        systemSleep(false); //only wake on interrupt
        break;  

      case 'j': //switch off controller
        systemOff();
        break;

      default: //unmatched character - send error and pretend the function call never happened
        Serial.print(incoming_serial + " does not have a matching function");
        system_idle = true;
    }
  }
}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  END OF STACK FUNCTIONS                                                                              //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

boolean stackCancelled(){ /* CHECK IF THE MAIN BUTTON HAS BEEN PUSHED DURING STACK TO CANCEL */

  if(!start_stack){
    return true;
  }
  
  return false; 

}

void stackEnd(){ /* RUN POST STACK CLEANUP THEN GO BACK TO MENU SECTION */

  screenUpdate();
  start_stack == false ? screen.print("Stack cancelled") : screen.print("Stack finished"); 
  delay(2000);

  if (return_to_start){   
    stepperDriverDirection("backwards"); //set the stepper direction for backward travel
    
    float exact_return_steps = slice_size * slice_counter * hardware_calibration_settings[active_hardware_calibration] * unit_of_measure_multipliers[active_unit_of_measure_multiplier] * available_micro_stepping[active_micro_stepping];
    int rounded_return_steps = (int) exact_return_steps - 0.5;
    
    screenUpdate();
    screen.print("Returning");
    
    stepperDriverEnable();

    for (int i; i < rounded_return_steps; i++){

      stepperDriverStep();
      if(!stepperDriverInBounds()){
       break; 
      }

    }

    if(!stepperDriverInBounds()) {
      stepperDriverClearLimitSwitch();
    }

    stepperDriverDisable(); 
  }

  menu_item = 1; //set menu to first option screen
  slice_counter = 0; //reset pic counter
  start_stack = false; //return to menu options section
  publish_update = true; //write the first menu item to the screen

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STEPPER MOTOR CONTROL FUNCTIONS                                                                     //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void stepperDriverClearLimitSwitch(){ /* PULL LINEAR STAGE AWAY FROM TRIPPED LIMIT SWITCH */

  stepperDriverEnable();
  screenUpdate();
  screen.print("End of travel!");
  screen.setPrintPos(0,4);
  screen.print("Returning...");

  if (mcp.digitalRead(limit_switch_front) == LOW){
    stepperDriverDirection("backwards"); //reverse stepper motor direction
    
    while (mcp.digitalRead(limit_switch_front) == LOW){ //turn stepper motor for as long as  the limit switch remains pressed 

      stepperDriverStep();
    
    }
    
    stepperDriverDirection("forwards"); //restore normal stepper motor direction
  }

  if (mcp.digitalRead(limit_switch_back) == LOW){
    stepperDriverDirection("forwards");//reverse stepper motor direction
    
    while (mcp.digitalRead(limit_switch_back) == LOW){ //turn stepper motor for as long as  the limit switch remains pressed 

      stepperDriverStep();

    }

  }

  screen.clearScreen();
  publish_update = true;
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

  stepper_driver_disable == true ? mcp.digitalWrite(stepper_driver_enable, HIGH) : mcp.digitalWrite(stepper_driver_enable, LOW);

}

void stepperDriverDirection(String direction){ /* SET STEPPER MOTOR DIRECTION */

  direction == "backwards" ? digitalWrite(stepper_driver_direction, LOW) : digitalWrite(stepper_driver_direction, HIGH); //if supplied string is anything other than "backwards" default to forwards

}

boolean stepperDriverInBounds(){ /* CHECK IF LINEAR STAGE IS IN BOUNDS OF TRAVEL I.E. NEITHER LIMIT SWITCH IS TRIPPED */

  if(mcp.digitalRead(limit_switch_front) == HIGH && mcp.digitalRead(limit_switch_back) == HIGH){
    return true;
  }
  
  return false;

}

void stepperDriverManualControl(){ /* MOVE STAGE BACKWARD AND FORWARD USING PUSH BUTTONS */

  if ((mcp.digitalRead(stepper_driver_forward) == LOW || incoming_serial == 'f') && stepperDriverInBounds()) {
    screenUpdate();
    screen.print("Moving forwards");
    screen.setPrintPos(0,4);
    screen.print(">-->>-->>-->>-->");

    while ((mcp.digitalRead(stepper_driver_forward) == LOW || incoming_serial == 'f') && stepperDriverInBounds()) {

      stepperDriverEnable();
      stepperDriverDirection("forwards");
      
      for (int i = 0; i<1; i++) {
      
        stepperDriverStep(); //move forwards
      
      }

      stepperDriverDisable();

    }

  }

  if ((mcp.digitalRead(stepper_driver_forward) == LOW || incoming_serial == 'g') && stepperDriverInBounds()) {
    screenUpdate();
    screen.print("Moving backwards");
    screen.setPrintPos(0,4);
    screen.print("<--<<--<<--<<--<");
    
    while ((mcp.digitalRead(stepper_driver_forward) == LOW || incoming_serial == 'g') && stepperDriverInBounds()) {
    
      stepperDriverEnable();
      stepperDriverDirection("backwards");
    
      for (int i = 0; i<1; i++) {
    
        stepperDriverStep(); //move backwards
    
      }
    
      stepperDriverDisable();
    }
    
  }

  if(!stepperDriverInBounds()) {
    stepperDriverClearLimitSwitch();
  }

  publish_update = true; //go back to displaying the active menu item once manual control button is no longer pressed

}

void stepperDriverMicroStepping(int step_setting){ /* SET DEGREE OF MICROSTEPPING TO USE */

  switch(step_setting){

    case 1: //full step (no microstepping)

      mcp.digitalWrite(stepper_driver_ms1, LOW);
      mcp.digitalWrite(stepper_driver_ms2, LOW);
      mcp.digitalWrite(stepper_driver_ms3, LOW);

      break;

    case 2: //half step

      mcp.digitalWrite(stepper_driver_ms1, HIGH);
      mcp.digitalWrite(stepper_driver_ms2, LOW);
      mcp.digitalWrite(stepper_driver_ms3, LOW);

      break;

    case 4: //quarter step

      mcp.digitalWrite(stepper_driver_ms1, LOW);
      mcp.digitalWrite(stepper_driver_ms2, HIGH);
      mcp.digitalWrite(stepper_driver_ms3, LOW);

      break;

    case 8: //eighth step

      mcp.digitalWrite(stepper_driver_ms1, HIGH);
      mcp.digitalWrite(stepper_driver_ms2, HIGH);
      mcp.digitalWrite(stepper_driver_ms3, LOW);

      break;

    case 16: //sixteenth step

      mcp.digitalWrite(stepper_driver_ms1, HIGH);
      mcp.digitalWrite(stepper_driver_ms2, HIGH);
      mcp.digitalWrite(stepper_driver_ms3, HIGH);

      break;

  }

}

void stepperDriverStep(){ /* SEND STEP SIGNAL TO A4988 STEPPER DRIVER */

  digitalWrite(stepper_driver_do_step, LOW); //this LOW to HIGH change is what creates the
  digitalWrite(stepper_driver_do_step, HIGH); //"Rising Edge" so the easydriver knows to when to step
  delayMicroseconds(step_delay); //delay time between steps, too fast and motor stalls

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  CHANGE SETTINGS IN USER MENUS                                                                       //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void settingUpdate(int &var, int lower, int upper, int multiplier = 1){ /* CHANGE THE ACTIVE MENU VARIABLE'S VALUE USING THE ENCODER */
  
  var = constrain(var, lower, upper); //keep variable value within specified range 

  if(setting_changed){
    var += (multiplier * setting_changed); //add or subtract from variable
    setting_changed = 0;
    publish_update = true;
  } 

}

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  USER MENUS - CHANGE, SAVE THEN DISPLAY SETTINGS                                                      //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/

void menuInteractions(){
  
  if (traverse_menus == true) { //use encoder to scroll through menu items

    settingUpdate(menu_item, 0, 12); //display the currently selected menu item

    //create looping navigation
    if(menu_item == 12) menu_item = 1;
    if(menu_item == 0) menu_item = 11;

  }

  switch (menu_item) { //the menu options

    case 1: //this menu item changes the number of increments to move each time

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(slice_size, 1, 1000);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        screenPrintCentre(String(slice_size));
        publish_update = false;
      }    

      break;

    case 2: //this menu item changes the number of slices to create in the stack

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(slices, 10, 5000, 10);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]); 
        screenPrintCentre(String(slices));
        publish_update = false;
      } 

      break;

    case 3: //this menu item changes the number of seconds to wait for the camera to take a picture before moving again - 
      //you may want longer if using flashes for adequate recharge time or shorter with continuous lighting
      //to reduce overall time taken to complete the stack

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(camera_pause, 1, 60);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        String camera_pause_string = String(camera_pause) += "s";
        screenPrintCentre(camera_pause_string);
        publish_update = false;
      }

      break;

    case 4: //this menu item screen toggles mirror lockup for the camera

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(mirror_lockup, 0, 1);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        mirror_lockup == true ? screenPrintCentre("Enabled") : screenPrintCentre("Disabled");          
        publish_update = false;
      }      

      break;

    case 5: //this menu item changes the number of images to take per focus slice (exposure bracketing)

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(camera_bracket, 1, 10);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        screenPrintCentre(String(camera_bracket));           
        publish_update = false;
      }      

      break;

    case 6: //this menu item toggles whether camera/subject is returned the starting position at the end of the stack

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(return_to_start, 0, 1);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        return_to_start == true ? screenPrintCentre("Enabled") : screenPrintCentre("Disabled");
        publish_update = false;
      }

      break; 

    case 7: //this menu items selects the unit of measure to use for focus slices: Microns, Millimimeters or Centimeteres

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(active_unit_of_measure_multiplier, 0, 2);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        screenPrintCentre(String(menu_strings_uom[active_unit_of_measure_multiplier]));  
        publish_update = false;
      }

      break; 

    case 8: //this menu item adjusts the stepper motor speed (delay in microseconds between slice_size)
      //setting this too low may cause the motor to begin stalling or failing to move at all

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(step_delay, 1000, 8000);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        String step_delay_string = String(step_delay) += "uS";
        screenPrintCentre(step_delay_string);  
        publish_update = false;
      }

      break; 

    case 9: //this menu item adjusts the degree of microstepping made by the a4988 stepper driver
    //more microsteps give the best stepping resolution but may require more power for consistency and accuracy
      
      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(active_micro_stepping, 0, 4);
        stepperDriverMicroStepping(available_micro_stepping[active_micro_stepping]);
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged
        screenUpdate(2, 0, menu_strings[menu_item]);
        String microstepping_string = "1/" + String(available_micro_stepping[active_micro_stepping]);
        screenPrintCentre(microstepping_string);  
        publish_update = false;
      }

      break;

    case 10: //this menu item screen selects the active hardware calibration constant

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(active_hardware_calibration, 0, 1); //final argument should be index of last entry in hardware_calibration_settings[]
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged

        screenUpdate(2, 0, menu_strings[menu_item]);

        switch(active_hardware_calibration){

          case 0:
            screenPrintCentre("Studio");
            break;

          case 1:
            screenPrintCentre("Field");
            break;

        }
            
        publish_update = false;

      }      

      break;

    case 11: //this menu item screen toggles power to an external 3.3v bluetooth board e.g. HC-05

      if (traverse_menus == false) { //press rotary encoder button within this menu item to edit variable
        settingUpdate(bluetooth_enabled, 0, 1);
        bluetoothTogglePower();
      }

      if (publish_update){ //only write to the screen when a change to the variables has been flagged

        screenUpdate(2, 0, menu_strings[menu_item]);
        
        if(bluetooth_enabled){
          bluetooth_connected == true ? screenPrintCentre("Connected") : screenPrintCentre("Unconnected");  
        } else {
          screenPrintCentre("Disabled");
        }
                    
        publish_update = false;

      }      

      break;

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

  if (!start_stack){ //this section allows manual control and configures settings using a simple screen menu system

    serialCommunications();

    if(mcp_interrupt_fired) handleMcpInterrupt(); //if an interrupt fired from the mcp23017, find out which pin fired it and deal with it

    menuInteractions(); //change menu options and update the screen when changed
    cronJobs(); //run timer driven tasks

  } else { //end of setup menu section

/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  THE FOCUS STACK.                                                                                    //
//  STARTS WHEN THE MAIN PUSH BUTTON IS PRESSED, CANCELLED IF PRESSED AGAIN                             // 
//  THE LOOP ADVANCES THE STAGE, TAKES A PICTURE AND REPEATS.                                           //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/ 

    stepperDriverDisable(); //disable the stepper driver when not in use to save power
    cameraProcessImages(); //take the image(s) for the first slice of the stack

    float exact_steps = slice_size * hardware_calibration_settings[active_hardware_calibration] * unit_of_measure_multipliers[active_unit_of_measure_multiplier] * available_micro_stepping[active_micro_stepping];
    int rounded_steps = (int) exact_steps - 0.5;

    for (int i = 0; i < slices -1; i++){ //for each subsequent focus slice in the stack...

      slice_counter++; //count slices made in the stack so far

      if(stackCancelled()){ //exit early if the stack has been cancelled
       break;
      }

      //print the current position in the stack to the screen
      screenPrintProgress();
      screen.setPrintPos(0,4);
      screen.print("Advance ");
      screen.print(slice_size);
      screen.print(menu_strings_uom[active_unit_of_measure_multiplier]);

      if(stackCancelled()){ //exit early if the stack has been cancelled
       break;
      }

      stepperDriverEnable(); //enable the A4988 stepper driver
      stepperDriverDirection("forwards"); //set the stepper direction for forward travel
      pause(100);

      for (int i = 0; i < rounded_steps; i++){ 

        stepperDriverStep(); //send a step signal to the stepper driver
              
        if(stackCancelled() || !stepperDriverInBounds()){ //exit early if the stack has been cancelled or a limit switch is hit
          break;
        }

      }

      if(!stepperDriverInBounds()) {
        stepperDriverClearLimitSwitch(); //clear hit limit switch
      }

      if(stackCancelled()){ //exit early if the stack has been cancelled
        break;
      }

      stepperDriverDisable(); //disable the stepper driver when not in use to save power  
      cameraProcessImages(); //take image(s)

    }

    stackEnd();

  } 
}
