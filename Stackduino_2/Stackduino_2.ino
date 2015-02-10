/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STACKDUINO 2 (BETA)                                                                                 //
//                                                                                                      //
//  https://github.com/ReallySmall/Stackduino-2                                                         //
//                                                                                                      //
//  An Arduino compatible Focus Stacking Controller optimised for mobile use                            //
//  Richard Iles 2014                                                                                   //
//                                                                                                      //
//  Written for use with the Digole 128 x 64 OLED module using default font (4 rows of 16 chars)       //
//  To support other fonts or screens, some recoding will be required                                    //            
////////////////////////////////////////////////////////////////////////////////////////////////////////*/



/* DEFINES AND DEPENDENCIES */
#define _Digole_Serial_I2C_ // OLED screen configured with solder jumper to run in I2C mode
#define ENC_A A0 // Rotary encoder
#define ENC_B A1 // Rotary encoder
#define ENC_PORT PINC // Rotary encoder

#include "DigoleSerial.h" // https://github.com/chouckz/HVACX10Arduino/blob/master/1.0.1_libraries/DigoleSerial/DigoleSerial.h
#include "Wire.h" //
#include "Adafruit_MCP23017.h" //
#include "ArduinoJson.h"
#include "SPI.h"
#include "SD.h"

/* CREATE REQUIRED OBJECTS */                                                                       
Adafruit_MCP23017 mcp; // 16 pin IO port expander
DigoleSerialDisp screen(&Wire,'\x27');



/* SET UP REMAINING ATMEGA PINS */                                                                          
const byte mcp_interrupt = 2; // Incoming MCP23017 interrupts
const byte button_main = 3;  // Main start/ stop stack button
const byte stepper_driver_direction = 4; // Direction pin on A4988 stepper driver
const byte stepper_driver_do_step = 5; // Step pin on A4988 stepper driver
const byte camera_focus = 6; // Camera autofocus signal
const byte camera_shutter = 7; // Camera shutter signal
const byte sd_ss =10; // SPI slave select for SD card slot
const byte batt_sense = A2; // Battery voltage sampling
const byte batt_fet = 17; // Connect/ disconnect battery voltage sampling line
const byte ext_analogue_1 = A6; // Analogue pin (supports analogueRead() only) broken out through DB15 - currently unused
const byte ext_analogue_2 = A7; // Analogue pin (supports analogueRead() only) broken out through DB15 - currently unused



/*  SET UP MCP23017 PINS */                                                                                      
const byte stepper_driver_forward = 0; // For manual stage positioning
const byte stepper_driver_backward = 1; // For manual stage positioning
const byte power_in_status = 2; // LTC4412 stat pin - indicates whether controller is currently powered by wall adapter or battery
const byte switch_off_flag = 3; // LTC2950 int pin - signals to the controller that it is about to be switched off
const byte switch_off = 4; // LTC2950 kill pin - disables 3.3v regulator and switches off controller
const byte limit_switch_front = 5; // Limit switch to stop stepper motor if end of travel is reached
const byte limit_switch_back = 6; // Limit switch to stop stepper motor if end of travel is reached
const byte bluetooth_toggle = 7; // Toggles power to bluetooth breakout board header
const byte ext_digital_1 = 8; // Digital pin broken out through DB15 - currently unused
const byte ext_digital_2 = 9;// Digital pin broken out through DB15 - currently unused
const byte button_rotary = 10; // Select/ unselect menu item button
const byte stepper_driver_ms1 = 11; // Toggles A4988 stepper driver MS1 pin to control degree of microstepping
const byte stepper_driver_ms2 = 12; // Toggles A4988 stepper driver MS2 pin to control degree of microstepping
const byte stepper_driver_ms3 = 13; // Toggles A4988 stepper driver MS3 pin to control degree of microstepping
const byte stepper_driver_enable = 14; // Disable/enables A4988 stepper driver to conserve power
const byte stepper_driver_sleep = 15; // Sleep/wake A4988 stepper driver to conserve power



/* SETTINGS */                                                                                                   
const char* menu_strings[] = {"Slice size", 
                              "Number of slices", 
                              "Pause time", 
                              "Mirror lockup", 
                              "Bracketing", 
                              "Return to start", 
                              "Unit of measure", 
                              "Stepper speed", 
                              "Microstepping", 
                              "Linear stage", 
                              "Bluetooth"};

const char* unit_of_measure_strings[] = {"mn", "mm", "cm"}; // Unit of measure strings for printing

// Current chars just fot testing, to be replaced with system fonts
char* system_icons[] = {"BO", // Bluetooth on
                              "AC", // Application connected
                              "AU", // Application unconnected
                              "NL", // Navigate left
                              "NR", // Navigate right
                              "SI", // Setting increment
                              "SD"}; // Setting decrement

const int unit_of_measure_multipliers[] = {1, 1000, 10000}; // Multipliers for active unit of measure (mn, mm or cm)
const byte micro_stepping[2][5][3] = {{{1},       {2},       {4},       {8},       {16}}, // Degrees of microstepping the A4988 stepper driver can use
                                      {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 1, 1}}}; // HIGH/ LOW combinations to set A4988 MSx pins at

// The controller relies on the assumption that 1 full step made by the stepper motor will move the 
// linear stage by 1 (or as close as possible to 1) micron.
// To achieve this relationship, the number of step signals generated should be multiplied by a constant 
// (obtained through a little maths and experimentation) which is unique to your hardware.
// As Stackduino could be used on more than one linear stage (e.g. studio and field setups), which each require
// unique constants, these are stored in an array:
const float hardware_calibration_settings[] = {1, 1};
const int resistor_1 = 10000; // Battery monitor voltage divider R1
const int resistor_2 = 3300; // Battery monitor voltage divider R2

volatile boolean start_stack = false; // False when in the menu, true when stacking
volatile boolean traverse_menus = true; // True when scrolling through menus, false when changing a menu item's value
volatile boolean mcp_interrupt_fired = false; // True whenever a pin on the mcp23017 with a listener went low
volatile int button_main_reading, button_rotary_reading; // The current reading from the button
volatile int button_main_previous = LOW, button_rotary_previous = LOW; // The previous reading from the button
volatile long button_main_time = 0, button_rotary_time = 0; // The last time the button was toggled

boolean application_connected = false; // Whether a remote application is actively communicating with the controller
boolean stepper_driver_disable = true; // Whether to disable the A4988 stepper driver when possible to save power and heat
boolean system_idle = true; // True until the controller is interacted with
boolean publish_update = true; // True whenever functions are called which change a setting's value

float calculated_voltage_reading = 0; // The actual battery voltage as calculated through the divider

// Vars employed by user menu settings
//
// Stored in array to facilitate easy retrival and saving to media
//
// [0] => Current value of setting 
// [1] => Lower bound of setting
// [2] => Upper bound of setting
// [3] => Multiplier to apply to setting value
//
int menu_settings[11][4] = {{10, 1, 1000, 1}, // "Slice size"
                            {10, 10, 5000, 10}, // "Number of slices"
                            {5, 1, 60, 1}, // "Pause time"
                            {0, 0, 1, 1}, // "Mirror lockup"
                            {1, 1, 10, 1}, // "Bracketing"
                            {0, 0, 1, 1}, // "Return to start"
                            {0, 0, 2, 1}, // "Unit of measure"
                            {2000, 1000, 8000, 1}, // "Stepper speed"
                            {4, 0, 4, 1}, // "Microstepping"
                            {0, 0, 1, 1}, // "Linear stage"
                            {1, 0, 1, 1}}; // "Bluetooth"

// Vars employed by system (not editable in user menus)
int menu_item = 0; // The active menu item
int setting_changed = 0; // Count increments the active menu setting should be changed by on the next poll
int incoming_serial; // Store latest byte from incoming serial stream
int system_timeout_off = 20; // Minutes of inactivity to wait until controller switches itself off - set to 0 to disable
int minutes_elapsed = 0; // Number of minutes elapsed
int seconds_counter = 0; // Number of seconds elapsed
int slice_counter = 0; // Count of number of focus slices made so far in the stack
char* application_connection_icon = "";
File settings_file;



/* SETUP() */                                                                                                      
void setup() {



  /* SET ATMEGA PINMODES AND PULLUPS */      
  pinMode(mcp_interrupt, INPUT_PULLUP);
  pinMode(button_main, INPUT_PULLUP); 
  pinMode(ENC_A, INPUT_PULLUP);  
  pinMode(ENC_B, INPUT_PULLUP);
  pinMode(stepper_driver_direction, OUTPUT); digitalWrite(stepper_driver_direction, LOW); 
  pinMode(stepper_driver_do_step, OUTPUT); digitalWrite(stepper_driver_do_step, LOW);     
  pinMode(camera_focus, OUTPUT); digitalWrite(camera_focus, LOW);
  pinMode(camera_shutter, OUTPUT); digitalWrite(camera_shutter, LOW);
  pinMode(batt_fet, OUTPUT); digitalWrite(batt_fet, LOW);
  pinMode(sd_ss, OUTPUT);

  

  /* START MCP23017 AND SET PINMODES AND PULLUPS */       
  mcp.begin(); // Using default address 0

  byte mcp_input_pins[] = {0,1,2,3,5,6,10};
  
  for(byte i=0; i < 7; i++){

    mcp.pinMode(mcp_input_pins[i], INPUT);
    mcp.pullUp(mcp_input_pins[i], HIGH);

  }

  byte mcp_output_pins[] = {4,7,8,9,11,12,13,14,15};
  
  for(byte i=0; i < 9; i++){

    mcp.pinMode(mcp_output_pins[i], OUTPUT);
    mcp.digitalWrite(mcp_output_pins[i], HIGH);

  }

  mcp.digitalWrite(14, LOW); // The A4988 enable pin needs setting LOW


  
  /* SET UP INTERRUPTS */       
  attachInterrupt(0, mcpInterrupt, FALLING); // ATMega external interrupt 0
  attachInterrupt(1, buttonMainToggle, FALLING); // ATMega external interrupt 1

  pciSetup(ENC_A); // ATMega pin change interrupt
  pciSetup(ENC_B); // ATMegapin change interrupt 
  
  mcp.setupInterrupts(true, false, LOW); // MCP23017 interupts (trigger ATMega external interrupt 0)
  mcp.setupInterruptPin(stepper_driver_forward, FALLING);
  mcp.setupInterruptPin(stepper_driver_backward, FALLING);
  mcp.setupInterruptPin(switch_off_flag,FALLING);
  mcp.setupInterruptPin(power_in_status,FALLING);
  mcp.setupInterruptPin(button_rotary,FALLING);   



  /* FINAL PRE-START CHECKS AND SETUP */       
  Serial.begin(9600); // Start serial (always initialise at 9600 for compatibility with OLED)
  if (!SD.begin(sd_ss)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  loadSettings(); // Attempt to load settings from SD card
  stepperDriverClearLimitSwitch(); // Make sure neither limit switch is hit on startup - rectify if so
  batteryMonitor(); // Check the battery level
  
}



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS                                                                                            //       
////////////////////////////////////////////////////////////////////////////////////////////////////////*/


/* SAVED SETTINGS
*
* Load settings from SD card/ save settings to SD card
* Stored in JSON format
*
*/
void loadSettings(){
  
  // Get the JSON settings string from the SD card
  settings_file = SD.open("settings.txt");
  if (settings_file) {
    char json_settings[400];
    while (settings_file.available()) {
        json_settings += settings_file.read());
    }
    // close the file:
    settings_file.close();
    
    StaticJsonBuffer<400> jsonBuffer;

  // Hardcoded JSON string for testing
  char json_settings[] = "{\"Slice size\":[20, 1, 1000, 1], \"Number of slices\":[30, 10, 5000, 10], \"Pause time\":[1, 1, 60, 1], \"Mirror lockup\":[1, 1, 0, 1], \"Bracketing\":[3, 1, 10, 1]], \"Return to start\":[1, 0, 1, 1], \"Unit of measure\":[1, 0, 2, 1]], \"Stepper speed\":[4000, 1000, 8000, 1], \"Microstepping\":[3, 0, 4, 1], \"Linear stage\":[1, 0, 1, 1], \"Bluetooth\"}:[0, 0, 1, 1]}";

  // Attempt to create a parseable object from the JSON string
  JsonObject& load_root = jsonBuffer.parseObject(json_settings);

  if (!load_root.success()) {
    Serial.println("parseObject() failed");
    return;
  } else {

    // If successful, loop through menu settings and assign variable values from parsed JSON object
    for(int i = 0; i < 11; i++){
      for(int j = 0; j < 4; j++){
        menu_settings[i][j] = load_root[menu_strings[i]][j];
      }
    }
    
    Serial.print("done");
    //saveSettings();

  }
  
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening settings.txt");
  } 

}

void saveSettings(){
  
    StaticJsonBuffer<50> jsonBuffer;
  
    // Create a JSON object to save settings into
    JsonObject& save_root = jsonBuffer.createObject();

    // Loop through menu settings and populate JSON object
    for(int i = 0; i < 11; i++){
      JsonArray& data = save_root.createNestedArray(menu_strings[i]);
      for(int j = 0; j < 4; j++){
        data.add(menu_settings[i][j]);
      }
    }

    save_root.prettyPrintTo(Serial);


    // Write JSON string to SD card
    // TODO

}



/* POWER OFF 
*
* The controller powers itself off by pulling the enable pin on the voltage regulator low
*
*/
void systemOff(){ 

  //mcp.digitalWrite(switch_off, LOW);

}



/* UPDATE THE SCREEN WHEN DATA CHANGES 
*
* Clears the screen then re-outputs it
*
* print_pos_y => Which text line to set the initial print position on 
* print_pos_x => Which character on the text line to set the initial print position on
* text => Optional text string to print
*
*/
void screenUpdate(int print_pos_y = 2, int print_pos_x = 0, const char text[16+1] = ""){ /* WIPE THE SCREEN, PRINT THE HEADER AND SET THE CURSOR POSITION */

  screen.clearScreen(); // Clear the screen
  screen.setPrintPos(0,0); // Set text position in top left corner
  //start_stack == true ? screen.print("Stack") : screen.print("Menu"); // Display whether menu or stack is active
  screen.print(application_connection_icon);
  screen.setPrintPos(3,0); // Set text position in top left corner
  screenPrintBluetooth(); // Display an icon for current bluetooth connection state
  screen.drawBox(1,15,128,1); // Draw a horizontal line to mark out a header section
  screenPrintPowerSource(); // Display the current power source in top right corner
  screenPrintMenuArrows(); // Print menu arrows (if a stack is not running)
  screen.setPrintPos(print_pos_x, print_pos_y); // Set text position to beginning of content area
  if(text){
    screen.print(text); // Print any supplied text string
  }
  system_idle = false; // Flag that the system was recently used

}



/* BATTERY VOLTAGE READING 
*
* Uses a mosfet to connect an analogue pin to the battery through a voltage divider
* 12v Max input for battery 
*
*/      
void batteryMonitor(){

  if(mcp.digitalRead(power_in_status) == HIGH){ // If currently running on battery

    static float raw_voltage_reading = 0; // Raw anologue reading of current battery voltage
    static float voltage_denominator = (float)resistor_2 / (resistor_1 + resistor_2);
    static float last_calculated_voltage_reading = 0.00; // The last battery reading

    digitalWrite(batt_fet, HIGH); // Connect the battery voltage line for reading
    raw_voltage_reading = analogRead(batt_sense); // Get an analogue reading of the battery voltage through the divider
    calculated_voltage_reading = ((raw_voltage_reading / 1023) * 3.3) / voltage_denominator; // Calculate the actual voltage
    digitalWrite(batt_fet, LOW); // Disconnect the battery voltage line again
    
    if(calculated_voltage_reading != last_calculated_voltage_reading){
      publish_update = true; // Flag the screen as updatable to display newest battery voltage reading
    } 
    
    last_calculated_voltage_reading = calculated_voltage_reading;
    
  }
}



/* BLUETOOTH PORT POWER
*
* Requires a compatible bluetooth breakout board (e.g. HC-05) connected to header H_1
* When enabled the VCC pin on H_1 is switched on via mosfet to power the breakout board
*
*/      
void bluetoothTogglePower(){

  static int last_bluetooth_status;
  byte bluetooth_enabled = menu_settings[10][0];

  if(!start_stack && last_bluetooth_status != bluetooth_enabled){
    mcp.digitalWrite(bluetooth_toggle, bluetooth_enabled == 1 ? LOW : HIGH);
  }

  last_bluetooth_status = bluetooth_enabled;   

}



/* APPLICATION CONNECTION STATUS
*
* Sends a keep-alive token and expects one back the next time it is called
* Otherwise assumes the connection with the remote application has been lost
* Also sets the icon for current connection status
*
*/      
void applicationConnectionStatus(){

  static boolean last_application_connection_status;

  application_connection_icon = application_connected == true ? system_icons[1]: system_icons[2];

  if(application_connected != last_application_connection_status){
    publish_update = true;
  }

  last_application_connection_status = application_connected;

  Serial.print(F("k")); // Send token

}



/* MAIN BUTTON
*
* Starts or stops a focus stack
*
*/ 
void buttonMainToggle(){
  
  system_idle = false;
  static int button_debounce = 400;
  button_main_reading = digitalRead(button_main);

  if (button_main_reading == LOW && button_main_previous == HIGH && millis() - button_main_time > button_debounce) {
    start_stack = start_stack == true ? false : true; 
    button_main_time = millis();    
  }

  button_main_previous = button_main_reading;

} 



/* ROTARY ENCODER BUTTON
*
* Toggles between menu navigation and variable editing
*
*/ 
void buttonRotaryToggle(){

  static int button_debounce = 400;
  button_rotary_reading = mcp.digitalRead(button_rotary);
  
  if (button_rotary_reading == LOW && button_rotary_previous == HIGH && millis() - button_rotary_time > button_debounce) {
    traverse_menus = traverse_menus == true ? false : true;
    button_rotary_time = millis();   
  }

  button_rotary_previous = button_rotary_reading;

} 



/* SIGNAL CAMERA TO TAKE IMAGE(S) 
*
* Optionally sends multiple delay seperated signals for exposure bracketing each focus slice 
*
*/
void cameraProcessImages(){ 

  for (int i = 1; i <= menu_settings[4][0]; i++){

    pause(1000); // Allow vibrations to settle

    if(menu_settings[4][0] > 1){ // If more than one image is being taken, display the current position in the bracket
      screenPrintProgress();
      screen.setPrintPos(0,4);
      screen.print(F("Bracket "));
      screen.print(i);
      screen.print(F("/"));
      screen.print(menu_settings[4][0]);
    }

    screenPrintProgress();
    screen.setPrintPos(0,4);
    screen.print(F("Pause for camera"));

    cameraShutterSignal(); // Take the image

    for(int i = 1; i <= menu_settings[2][0]; i++){

      pause(1000);
      screenPrintProgress();
      screen.setPrintPos(0,4);
      screen.print(F("Resume in "));
      screen.print(menu_settings[2][0] - i); // Print number of seconds remaining
      screen.print(F("s"));

      if(stackCancelled()) break; // Exit early if the stack has been cancelled

    }

    if(stackCancelled()) break; // Exit early if the stack has been cancelled

  }
}



/* TRIGGER THE CAMERA SHUTTER 
*
* Optionally sends two delay seperated signals to support mirror lockup
*
*/
void cameraShutterSignal() { 

  for (int i = 0; i <= menu_settings[3][0]; i++){

    if(menu_settings[3][0]){
      screenPrintProgress();
      screen.setPrintPos(0,4); 
      screen.print(i == 0 ? F("Raise mirror") : F("Shutter"));
      pause(500);
    }

    digitalWrite(camera_focus, HIGH); // Trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(camera_shutter, HIGH); // Trigger camera shutter

    pause(200); //Small delay needed for camera to process above signals

    digitalWrite(camera_shutter, LOW); // Switch off camera trigger signal
    digitalWrite(camera_focus, LOW); // Switch off camera focus signal

    if(menu_settings[3][0] && i == 0){
      pause(2000); // Pause between mirror up and shutter actuation
    }

    if(stackCancelled()) break; // Exit early if the stack has been cancelled 

  }
}



/* SYSTEM TASKS
*
* Switch Bluetooth on or off
* Check connection status of external applications
* Periodically check the battery level
* Switch off the controller if it has been unused for n minutes
*
*/                                       
void systemTasks(){

//  static unsigned long one_second = millis() + 1000; // 1 second from now
//  static boolean connection_status_polled = true;
//  
//  bluetoothTogglePower(); // Switch Bluetooth port on or off
//  
//  if(millis() >= one_second){ // Count each second
//    seconds_counter++;
//    one_second = millis() + 1000;
//    connection_status_polled = false;
//  }
//
//  if(seconds_counter % 5 == 0){ // Every 5 seconds
//    if(!connection_status_polled){
//      applicationConnectionStatus();
//      connection_status_polled = true;
//    }
//  }
//
//  if(seconds_counter == 60){ // Every 1 minute
//    
//    seconds_counter = 0; // Reset to count another minute
//    batteryMonitor(); // Check the battery level
//    
//    if(!system_idle){ // If the controller has been used in the past minute
//      minutes_elapsed = 0; // Restart the minutes counter
//      system_idle = true; // Reset the system flag
//    } else { //Otherwise a minute has passd with no activity
//      minutes_elapsed++; // Keep count of minutes elapsed
//    }
//
//  }
//
//  // If enabled after n minutes switch off the controller 
//  if(system_timeout_off && (minutes_elapsed >= system_timeout_off)){
//    systemOff();
//  }

}



/* RETURN ANY CHANGE IN ROTARY ENCODER POSITION 
* 
* Using work by:
* http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
*
*/
int8_t encoderRead(){

  const int8_t pulses_between_detents[] = {3, -3}; // Number of pulses between detents  

  static int8_t enc_states[] = { 
    0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0
    //0,1,-1,0,-1,0,0,1,1,0,0,-1,0,-1,1,0 // Use this line instead to increment encoder in the opposite direction 
  };

  static uint8_t old_AB = 0;

  old_AB <<= 2; // Remember previous state
  old_AB |= ( ENC_PORT & 0x03 ); // Add current state

  int8_t encoder_data = ( enc_states[( 0x0f & old_AB )]);

  static int8_t encoder_pulse_counter = 0; // Counts pulses from encoder

  if(encoder_data){ // If Gray Code returns a valid input (1 or -1) 
    encoder_data == 1 ? encoder_pulse_counter++ : encoder_pulse_counter--; 
  }

  if(encoder_pulse_counter > pulses_between_detents[0]){ // Record the encoder was moved by one click (detent)
    setting_changed++;
    encoder_pulse_counter = 0;
  } 

  else if(encoder_pulse_counter < pulses_between_detents[1]) { // Record the encoder was moved by one click (detent) in the opposite direction
    setting_changed--;
    encoder_pulse_counter = 0;
  }

}



/* SET UP ATMEGA PIN CHANGE INTERRUPTS */
void pciSetup(byte pin)
{
  *digitalPinToPCMSK(pin) |= bit (digitalPinToPCMSKbit(pin));  // enable pin
  PCIFR  |= bit (digitalPinToPCICRbit(pin)); // clear any outstanding interrupt
  PCICR  |= bit (digitalPinToPCICRbit(pin)); // enable interrupt for the group
}



/* READ THE ENCODER ON ANALOGUE PORT INTERRUPT */
ISR (PCINT1_vect){

  encoderRead();

}  



/* FLAG NEW INTERRUPTS FROM THE MCP23017 */
void mcpInterrupt(){ 

  mcp_interrupt_fired = true;
  system_idle = false;

}



/* PROCESS FLAGGED INTERRUPTS FROM THE MCP23017 */
void handleMcpInterrupt(){

  detachInterrupt(0); // Detach the interrupt while the current instance is dealt with

  uint8_t pin = mcp.getLastInterruptPin(); // Find out which pin on the mcp created the interrupt
  uint8_t val = mcp.getLastInterruptPinValue(); // And get its current state

  if(pin == switch_off_flag && val == LOW){ // If a switchoff signal was recieved from the pushbutton controller
    systemOff();
  }

  if(pin == stepper_driver_forward && start_stack == false){ // If a manual stage control button was pressed
    stepperDriverManualControl();
  }
  
  if(pin == stepper_driver_backward && start_stack == false){ // If a manual stage control button was pressed
    stepperDriverManualControl();
  }

  if((pin == limit_switch_front && val == LOW) || (pin == limit_switch_back && val == LOW)){ // If a limit switch was hit, clear it
    stepperDriverClearLimitSwitch();
  }

  if(pin == power_in_status){ // If the power source changed, update the screen
    publish_update = true;
  }

  if(pin == button_rotary){ // Check the current state of the rotary encoder button
    buttonRotaryToggle();
  }

  EIFR=0x01; // Clear interrupts
  mcp_interrupt_fired = false; // Reset the flag
  attachInterrupt(0, mcpInterrupt, FALLING); // Re-attach the interrupt

}



/* PAUSE
*
* Delays are needed throughout the code, but these should be non-blocking
*        
*/
void pause(int length){

  unsigned long initial_millis = millis(); // Get the current millis
  unsigned long end_millis = initial_millis + length; // Define when the pause ends

  while(end_millis > millis()){ // If the end of the pause time hasn't yet been reached

    if(stackCancelled()) break; // Check continuously for any reason to end it early
       
  }

}



/* PRINT MENU NAVIGATION ARROWS ON DIGOLE OLED SCREEN 
*
* Assumes the default font is in use (allowing 4 rows of 16 characters) 
*
*/
void screenPrintMenuArrows(){ 

//  if(!start_stack) {
//    if(traverse_menus) { // Move to the next menu item 
//
//      screen.drawBox(5,54,1,9); // Left outward arrow
//      screen.drawBox(4,55,1,7);
//      screen.drawBox(3,56,1,5);
//      screen.drawBox(2,57,1,3);
//      screen.drawBox(1,58,1,1);
//
//      screen.drawBox(122,54,1,9); // Right outward arrow
//      screen.drawBox(123,55,1,7);
//      screen.drawBox(124,56,1,5);
//      screen.drawBox(125,57,1,3);
//      screen.drawBox(126,58,1,1);
//      
//    } else { // Change the value of the current menu item
//
//      screen.drawBox(1,54,1,9); // Left inward arrow
//      screen.drawBox(2,55,1,7);
//      screen.drawBox(3,56,1,5);
//      screen.drawBox(4,57,1,3);
//      screen.drawBox(5,58,1,1);
//
//      screen.drawBox(122,58,1,1); // Right inward arrow
//      screen.drawBox(123,57,1,3);
//      screen.drawBox(124,56,1,5);
//      screen.drawBox(125,55,1,7);
//      screen.drawBox(126,54,1,9);
//
//    }
//  }

}



/* PRINT BLUETOOTH ICON ON DIGOLE OLED SCREEN 
*
* Assumes the default font is in use (allowing 4 rows of 16 characters) 
*
*/
void screenPrintBluetooth(){ 

  if(menu_settings[10][0] == 1){ //if the bluetooth header (H_1) is active
    screen.print(system_icons[0]);
  }

}



/* PRINT APPLICATION CONNECTION ICON ON DIGOLE OLED SCREEN 
*
* Assumes the default font is in use (allowing 4 rows of 16 characters) 
*
*/
void screenPrintConnection(){ 

  screen.print(application_connected == true ? system_icons[1] : system_icons[2]);

}



/* PRINT CENTERED TEXT ON DIGOLE OLED SCREEN 
*
* Assumes the default font is in use (allowing 4 rows of 16 characters) 
*
* text => The string to be centered and printed
* string_length => The length of the string in the char buffer
* print_pos_y => The text line to print on
*
*/
void screenPrintCentre(char* text, byte string_length, int print_pos_y = 4){ 

  byte offset = (16 - string_length) / 2; // Calculate the offset for the number of characters in the string

  screen.setPrintPos(offset, print_pos_y);

  if(string_length % 2 != 0){ // Dividing an odd numbered int by 2 discards the remainder, creating an offset which is half a text character width too short
    screen.setTextPosOffset(4, 0); // So the text needs to be nudged to the right a bit on a pixel level to centre it properly 
  } 

  screen.print(text); // Finally, print the centered text to the screen

}



/* PRINT THE ACTIVE POWER SOURCE ON DIGOLE OLED SCREEN 
*
* Assumes the default font is in use (allowing 4 rows of 16 characters) 
*
*/
void screenPrintPowerSource(){ 

//  if(mcp.digitalRead(power_in_status) == LOW){ // Draw plug symbol (dc adapter connected)
//      screen.drawBox(115,1,3,1);
//      screen.drawBox(112,2,7,1);
//      screen.drawBox(115,3,13,2);
//      screen.drawBox(112,5,7,1);
//      screen.drawBox(115,6,3,1);
//    } else { // Draw battery symbol and print current voltage
//      screen.setPrintPos(7,0);
//      screen.print(calculated_voltage_reading);
//      screen.print(F("v"));
//      screen.drawBox(112,3,2,2);
//      screen.drawBox(114,1,14,6);
//    }

  }



/* PRINT THE CURRENT POSITION IN A RUNNING FOCUS STACK ON DIGOLE OLED SCREEN 
*
* Assumes the default font is in use (allowing 4 rows of 16 characters) 
*
*/
void screenPrintProgress(){

  screenUpdate(); 
  screen.print(F("Slice "));
  screen.print (slice_counter);
  screen.print (F("/"));
  screen.print (menu_settings[1][0]);

}



/* CONTROL VIA SERIAL 
* 
* Checks for incoming characters over serial and runs any mtaching functions
* Allows for remote control via USB or Bluetooth
*
*/
void serialCommunications(){

  if (Serial.available() > 0) {

    system_idle = false;  
    int incoming_serial = Serial.read(); // read the incoming byte:

    switch(incoming_serial){

      case 'a': // Start or stop stack
        start_stack = start_stack == true ? false : true;
        break;

      case 'b': // Toggle menu navigation
        traverse_menus = traverse_menus == true ? false : true;
        break;

      case 'c': // Increment active setting variable
        setting_changed++;
        break;

      case 'd': // Decrement active setting variable
        setting_changed--;
        break;

      case 'e': // Move stage forward
        stepperDriverManualControl();
        break;

      case 'f': // Move stage backward
        stepperDriverManualControl();
        break;

      case 'g': // Take a test image
        cameraShutterSignal();
        break;

      case 'h': // Switch Bluetooth on or off
        menu_settings[10][0] = menu_settings[10][0] == 1 ? 0 : 1;
        break;

      case 'i': // 
        
        break;  

      case 'j': // Switch off controller
        systemOff();
        break;
        
      case 'k': // Keep connection alive
        application_connected = true;
        break;

      default: // Unmatched character - send error then pretend the function call never happened
      Serial.println(incoming_serial);
      Serial.print(" is not a recognised command");
      system_idle = true;
    }
  }
}



/* CHECK IF THE FOCUS STACK HAS BEEN CANCELLED */
boolean stackCancelled(){ 

  if(!start_stack){
    return true;
  }

  return false; 

}



/* ENABLE OR DISABLE THE STEPPER DRIVER AND SET DIRECTION 
*
* enable => Enables the stepper driver if true, Disables if false and option set
* direction => Set as 1 for forward direction, 0 for backward direction
*
*/
void stepperDriverEnable(boolean enable = true, byte direction = 1) { 

  if(enable){
    mcp.digitalWrite(stepper_driver_enable, LOW);
    digitalWrite(stepper_driver_direction, direction);
    stepperDriverMicroStepping();
  } else if(stepper_driver_disable){
    mcp.digitalWrite(stepper_driver_enable, HIGH);
  }

}



/* CLEAN UP AFTER FOCUS STACK COMPLETION 
*
* Optionally return the stage to its starting position
* Reset various variables to inital values
*
*/
void stackEnd(){

  screenUpdate();
  start_stack == false ? screen.print(F("Stack cancelled")) : screen.print(F("Stack finished")); 
  delay(2000);

  if (menu_settings[5][0]){
  
    stepperDriverEnable(true, 0);

    float exact_return_steps = menu_settings[0][0] * slice_counter * hardware_calibration_settings[menu_settings[9][0]] * unit_of_measure_multipliers[menu_settings[6][0]] * micro_stepping[0][menu_settings[8][1]][0];
    long rounded_return_steps = (long) exact_return_steps - 0.5;

    screenUpdate();
    screen.print(F("Returning"));
    delay(2000);

    stepperDriverEnable(true, 1);

    for (int i; i < rounded_return_steps; i++){

      stepperDriverStep();
      if(!stepperDriverInBounds()) break; 

    }

    if(!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
    stepperDriverEnable(false);

  }

  menu_item = 1; // Set menu to first option screen
  slice_counter = 0; // Reset pic counter
  start_stack = false; // Return to menu options section
  publish_update = true; // Write the first menu item to the screen

}



/* PULL STAGE AWAY FROM TRIPPED LIMIT SWITCH */
void stepperDriverClearLimitSwitch(){ 

  screenUpdate();
  screen.print(F("End of travel!"));
  screen.setPrintPos(0,4);
  screen.print(F("Returning..."));

  if (mcp.digitalRead(limit_switch_front) == LOW){
    stepperDriverEnable(true, 0); //reverse stepper motor direction

    while (mcp.digitalRead(limit_switch_front) == LOW){ //turn stepper motor for as long as  the limit switch remains pressed 

      stepperDriverStep();

    }

    stepperDriverEnable(true, 1); //restore normal stepper motor direction
  }

  if (mcp.digitalRead(limit_switch_back) == LOW){
    stepperDriverEnable(true, 1); 

    while (mcp.digitalRead(limit_switch_back) == LOW){ //turn stepper motor for as long as  the limit switch remains pressed 

      stepperDriverStep();

    }

  }

  screen.clearScreen();
  publish_update = true;
  stepperDriverEnable(false);

  if(start_stack){ // If a stack was in progress, cancel it
    start_stack = false;
    stackCancelled();
  } 

}



/* CHECK IF STAGE IS IN BOUNDS OF TRAVEL I.E. NEITHER LIMIT SWITCH IS TRIPPED */
boolean stepperDriverInBounds(){ 

  if(mcp.digitalRead(limit_switch_front) == HIGH && mcp.digitalRead(limit_switch_back) == HIGH){
    return true;
  }

  return false;

}



/* MOVE STAGE BACKWARD AND FORWARD */
void stepperDriverManualControl(){ 

  // Move stage forwards
  if (mcp.digitalRead(stepper_driver_forward) == LOW && stepperDriverInBounds()) {
    screenUpdate();
    screen.print(F("Moving forwards"));
    screen.setPrintPos(0,4);
    screen.print(F(">-->>-->>-->>-->"));
    stepperDriverEnable(true, 1);

    while (mcp.digitalRead(stepper_driver_forward) == LOW && stepperDriverInBounds()) {

      stepperDriverStep(); 

    }

  }

  // Move stage backwards
  if (mcp.digitalRead(stepper_driver_forward) == LOW && stepperDriverInBounds()) {
    screenUpdate();
    screen.print(F("Moving backwards"));
    screen.setPrintPos(0,4);
    screen.print(F("<--<<--<<--<<--<"));
    stepperDriverEnable(true, 0);

    while (mcp.digitalRead(stepper_driver_forward) == LOW && stepperDriverInBounds()) {

      stepperDriverStep(); 

    }

  }

  stepperDriverEnable(false);

  if(!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
  publish_update = true; // Go back to displaying the active menu item once manual control button is no longer pressed

}



/* SET DEGREE OF MICROSTEPPING FOR A4988 STEPPER DRIVER TO USE 
* 
* Loops through the active microstepping subarray and writes the A4988's MSx pins HIGH or LOW accordingly
* MSx pins are MCP23017 11 - 13 
*
*/
void stepperDriverMicroStepping(){

  int active_ms = menu_settings[8][1];

  for(byte i=0, j=11; i<2; i++, j++){

    mcp.digitalWrite(j, micro_stepping[1][active_ms][i]);

  }

}



/* SEND STEP SIGNAL TO A4988 STEPPER DRIVER */
void stepperDriverStep(){ 

  digitalWrite(stepper_driver_do_step, LOW); // This LOW to HIGH change is what creates the
  digitalWrite(stepper_driver_do_step, HIGH); // "Rising Edge" so the driver knows when to step
  delayMicroseconds(menu_settings[7][0]); // Delay time between steps, too short and motor may stall or miss steps

}



/* CHANGE THE ACTIVE MENU VARIABLE'S VALUE
*
* May be implemented by either the rotary encoder or a command via serial
*
* var => The variable to change the value of
* lower => The lower bounds for constraining the value of var
* upper => The upper bounds for constraining the value of var
* multipler = Factor to multiply the change in value of var by
*  
*/
void settingUpdate(int &var, int lower, int upper, int multiplier = 1){ 

  var = constrain(var, lower, upper); // Keep variable value within specified range 

  if(setting_changed){ // If the variable's value was changed at least once since the last check
    var += (multiplier * setting_changed); // Add or subtract from variable
    setting_changed = 0; // Reset for next check
    publish_update = true; // Update menus
  } 

}



/* OUTPUT MENUS
*
* Publishes menus which refresh whenever the selected variable changes
* 
*/
void menuInteractions(){

  if (traverse_menus) { // Move through menu items
    settingUpdate(menu_item, -1, 11); // Display the currently selected menu item
    if(menu_item == 11) menu_item = 0; // Create looping navigation
    if(menu_item == -1) menu_item = 10; // Create looping navigation
  } else { // Otherwise change the value of the current menu item
    settingUpdate(menu_settings[menu_item][0], menu_settings[menu_item][1], menu_settings[menu_item][2], menu_settings[menu_item][3]);
  }

  if (publish_update){ // Refresh menu content if the active variable has changed

    int menu_var = menu_settings[menu_item][0];
    char print_buffer[16 + 1];
    byte string_length;

    switch (menu_item) { // The menu options

      case 0: // Change the number of increments to move each time
        string_length = sprintf(print_buffer, "%d", menu_var);  
        break;

      case 1: // Change the number of slices to create in the stack
        string_length = sprintf(print_buffer, "%d", menu_var); 
        break;

      case 2: // Change the number of seconds to wait for the camera to capture an image before continuing 
        string_length = sprintf(print_buffer, "%ds", menu_var);
        break;

      case 3: // Toggle mirror lockup for the camera
        string_length = menu_var == 1 ? sprintf(print_buffer, "Enabled") : sprintf(print_buffer, "Disabled");      
        break;

      case 4: // Change the number of images to take per focus slice (exposure bracketing)
        string_length = sprintf(print_buffer, "%d", menu_var);   
        break;

      case 5: // Toggle whether camera/subject is returned the starting position at the end of the stack
        string_length = menu_var == 1 ? sprintf(print_buffer, "Enabled") : sprintf(print_buffer, "Disabled");
        break; 

      case 6: // Select the unit of measure to use for focus slices: Microns, Millimimeters or Centimeters
        string_length = sprintf(print_buffer, "%s", unit_of_measure_strings[menu_var]);
        break; 

      case 7: // Adjust the stepper motor speed (delay in microseconds between slice_size)
              // A smaller number gives faster motor speed but reduces torque
              // Setting this too low may cause the motor to miss steps or stall
        string_length = sprintf(print_buffer, "%duS", menu_var);
        break; 

      case 8: // Adjust the degree of microstepping made by the a4988 stepper driver
              // More microsteps give the best stepping resolution but may require more power for consistency and accuracy
        string_length = sprintf(print_buffer, "1/%d", micro_stepping[0][menu_var][0]);
        break;

      case 9: // Select the active hardware calibration constant
        string_length = menu_var == 0 ? sprintf(print_buffer, "Studio") : sprintf(print_buffer, "Field");     
        break;

      case 10: // Toggle power to an external 3.3v bluetooth board e.g. HC-05
        if(menu_settings[10][0] == 1){
          string_length = menu_var == 1 ? sprintf(print_buffer, "Connected") : sprintf(print_buffer, "Unconnected");
        } else {
          string_length = sprintf(print_buffer, "Disabled");
        }     
        break;

    }

    //saveSettings();
    screenUpdate(2, 0, menu_strings[menu_item]);
    screenPrintCentre(print_buffer, string_length);
    publish_update = false;

  }  

} 



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP                                                                                           //                                                                                                     
////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void loop(){



  /* MENUS AND STAGE CONTROL
  *
  * Manual control of linear stage is only enabled when in this section of the loop 
  * - prevents running stacks being ruined by accidental button presses
  * 
  * Menu navigation controlled either by rotary encoder with integral push button, or via serial
  *
  */
  if (!start_stack){ // User settings menus and manual stage control

    if(mcp_interrupt_fired) handleMcpInterrupt(); // Catch interrupts and run any required functions
    serialCommunications();  // Check for commands via serial
    menuInteractions(); // Change menu options and update the screen when changed
    systemTasks(); // Run timer driven tasks such as battery level checks and auto switch off

  } 



  /* FOCUS STACK
  *
  * Images are captured, the stage is advanced and the process is repeated for as many times as needed
  *
  */
  else {

    slice_counter++; // Register the first image(s) of the stack is being taken
    cameraProcessImages(); // Take the image(s) for the first slice of the stack

    float exact_steps = menu_settings[0][0] * hardware_calibration_settings[menu_settings[9][0]] * unit_of_measure_multipliers[menu_settings[6][0]] * micro_stepping[0][menu_settings[8][0]][0];
    long rounded_steps = (long) exact_steps - 0.5;

    for (int i = 1; i < menu_settings[1][0]; i++){ // For each subsequent focus slice in the stack

      slice_counter++; // Record slices made in the stack so far

      if(stackCancelled()) break; // Exit early if the stack has been cancelled

      screenPrintProgress(); // Print the current position in the stack
      screen.setPrintPos(0,4);
      screen.print(F("Advance "));
      screen.print(menu_settings[0][0]);
      screen.print(unit_of_measure_strings[menu_settings[6][0]]);

      if(stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(true, 1); // Enable the A4988 stepper driver

      for (int i = 0; i < rounded_steps; i++){ 

        stepperDriverStep(); // Send a step signal to the stepper driver     
        if(stackCancelled() || !stepperDriverInBounds()) break; // Exit early if the stack has been cancelled or a limit switch is hit

      }

      if(!stepperDriverInBounds()) stepperDriverClearLimitSwitch(); // Clear hit limit switch 
      if(stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(false); // Disable the stepper driver when not in use to save power  
      cameraProcessImages(); // Take image(s)

    }

    stackEnd(); // End of focus stack tasks

  } 
}
