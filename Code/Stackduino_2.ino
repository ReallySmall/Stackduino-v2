//////////////////////////////////////////////////////////////////////////////////////////////////////////
//  STACKDUINO 2 (BETA)                                                                                 //
//                                                                                                      //
//  https://github.com/ReallySmall/Stackduino-2                                                         //
//                                                                                                      //
//  An Arduino compatible Focus Stacking Controller optimised for mobile use                            //
//  Richard Iles 2014                                                                                   //
//                                                                                                      //
//  Written for use with the Digole 128 x 64 OLED module using default font (4 rows of 16 chars)        //
//  To support other fonts or screens, some recoding will be required                                   //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/



/* DEFINES AND DEPENDENCIES */
#define _Digole_Serial_I2C_ // OLED screen configured with solder jumper to run in I2C mode
#define OLED_ROWS 4 // OLED text rows
#define OLED_COLS 16 // OLED text columns
#define ENC_A A0 // Rotary encoder
#define ENC_B A1 // Rotary encoder
#define ENC_PORT PINC // Rotary encoder
#define settings_elements 4 // Number of properties in each settings

#include "DigoleSerial.h" // https://github.com/chouckz/HVACX10Arduino/blob/master/1.0.1_libraries/DigoleSerial/DigoleSerial.h
#include "Wire.h" //
#include "Adafruit_MCP23017.h" //
#include "SPI.h"
#include "SD.h"
#include "avr/pgmspace.h"



/* CREATE REQUIRED OBJECTS */
Adafruit_MCP23017 mcp; // 16 pin IO port expander
DigoleSerialDisp screen(&Wire, '\x27'); // Digole OLED in i2c mode

typedef struct { // A struct type for storing char arrays
  char title [OLED_COLS + 1];
} stringConstants;

const stringConstants settings_titles[] PROGMEM = { // A struct of char arrays stored in Flash
  {"Stackduino v2.2"},
  {"Slice size"},
  {"Slices"},
  {"Pause for"},
  {"Mirror lockup"},
  {"Bracketing"},
  {"Return to home"},
  {"Units"},
  {"Motor speed"},
  {"Microsteps"},
  {"Bluetooth"}
};

char uom_chars[3] = {'u', 'm', 'c'};

struct Settings { // A struct type for storing settings

  int value; // The setting value
  byte lower; // The lowest value the setting may have
  int upper; // The highest value the setting may have
  byte multiplier; // Any multiplier to apply when the setting is incremented

} settings[] = { // A struct of user menu settings
  {}, // The home screen - not actually used by any functions at the moment, but the placeholder array must exist to maintain the correct indexing
  {10, 10, 500, 1}, // "Slice size"
  {10, 10, 500, 10}, // "Number of slices"
  {5, 1, 60, 1}, // "Pause time"
  {0, 0, 1, 1}, // "Mirror lockup"
  {1, 1, 10, 1}, // "Bracketing"
  {0, 0, 1, 1}, // "Return to start"
  {0, 0, 2, 1}, // "Unit of measure"
  {2, 1, 8, 1}, // "Stepper speed"
  {4, 0, 4, 1}, // "Microstepping"
  {0, 0, 1, 1} // "Bluetooth"
};

byte settings_count = sizeof(settings) / sizeof(Settings); // The number of settings
char char_buffer[OLED_COLS + 1]; // make sure this is large enough for the largest string it must hold



/* SET UP REMAINING ATMEGA PINS */
const byte mcp_int = 2; // Incoming MCP23017 interrupts
const byte btn_main = 3;  // Main start/ stop stack button
const byte step_dir = 4; // Direction pin on A4988 stepper driver
const byte do_step = 5; // Step pin on A4988 stepper driver
const byte cam_focus = 6; // Camera autofocus signal
const byte cam_shutter = 7; // Camera shutter signal
const byte sd_ss = 10; // SPI slave select for SD card slot
const byte batt_sense = A2; // Battery voltage sampling
const byte batt_fet = 17; // Connect/ disconnect battery voltage sampling line
const byte alog_1 = A6; // Analogue pin (supports analogueRead() only) broken out through DB15 - currently unused
const byte alog_2 = A7; // Analogue pin (supports analogueRead() only) broken out through DB15 - currently unused



/*  SET UP MCP23017 PINS */
const byte step_bwd = 0; // For manual stage positioning
const byte step_fwd = 1; // For manual stage positioning
const byte pwr_stat = 2; // LTC4412 stat pin - indicates whether controller is currently powered by wall adapter or battery
const byte switch_off_flag = 3; // LTC2950 int pin - signals to the controller that it is about to be switched off
const byte switch_off = 4; // LTC2950 kill pin - disables 3.3v regulator and switches off controller
const byte limit_switch_front = 5; // Limit switch to stop stepper motor if end of travel is reached
const byte limit_switch_back = 6; // Limit switch to stop stepper motor if end of travel is reached
const byte bluetooth_pwr_toggle = 7; // Toggles power to bluetooth breakout board header
const byte connected_hardware = 8; // Digital pin broken out through DB15 - currently unused
const byte dig_1 = 9;// Digital pin broken out through DB15 - currently unused
const byte btn_rotary = 10; // Select/ unselect menu item button
const byte ms1 = 11; // Toggles A4988 stepper driver MS1 pin to control degree of microstepping
const byte ms2 = 12; // Toggles A4988 stepper driver MS2 pin to control degree of microstepping
const byte ms3 = 13; // Toggles A4988 stepper driver MS3 pin to control degree of microstepping
const byte step_enable = 14; // Disable/enables A4988 stepper driver to conserve power
const byte step_sleep = 15; // Sleep/wake A4988 stepper driver to conserve power



/* SETTINGS */
const int uom_multipliers[] = {1, 1000, 10000}; // Multipliers for active unit of measure (mn, mm or cm)
const byte micro_stepping[2][5][3] = {{{1},       {2},       {4},       {8},       {16}}, // Degrees of microstepping the A4988 stepper driver can use
  {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {1, 1, 1}}
}; // HIGH/ LOW combinations to set A4988 MSx pins at

float active_hardware_calibration_setting;

volatile boolean start_stack = false; // False when in the menu, true when stacking
volatile boolean traverse_menus = true; // True when scrolling through menus, false when changing a menu item's value
volatile boolean mcp_int_fired = false; // True whenever a pin on the mcp23017 with a listener went low
volatile byte btn_reading; // The current reading from the button
volatile byte btn_previous = LOW; // The previous reading from the button
volatile long btn_time = 0; // The last time the button was toggled

boolean app_connected = false; // Whether a remote application is actively communicating with the controller
boolean can_disable_stepper = true; // Whether to disable the A4988 stepper driver when possible to save power and heat
boolean idle = true; // True until the controller is interacted with
boolean update_display = true; // True whenever functions are called which change a setting's value
boolean update_header = true; // True whenever the value of an item in the header changes
boolean home_screen = true;
boolean previous_direction;

float calc_volts = 0.00; // The actual battery voltage as calculated through the divider

int menu_item = 0; // The active menu item
int increments = 0; // Count increments the active menu setting should be changed by on the next poll
byte sys_off = 20; // Minutes of inactivity to wait until controller switches itself off - set to 0 to disable
byte slice_count = 0; // Count of number of focus slices made so far in the stack

unsigned long time_stack_started;

char* app_conn_icon = "";

File settings_file;



/* SETUP() */
void setup() {

  /* SET ATMEGA PINMODES AND PULLUPS */
  pinMode(mcp_int, INPUT); digitalWrite(mcp_int, HIGH);
  pinMode(btn_main, INPUT); digitalWrite(btn_main, HIGH);
  pinMode(ENC_A, INPUT); digitalWrite(ENC_A, HIGH);
  pinMode(ENC_B, INPUT); digitalWrite(ENC_B, HIGH);
  pinMode(step_dir, OUTPUT); digitalWrite(step_dir, LOW);
  pinMode(do_step, OUTPUT); digitalWrite(do_step, LOW);
  pinMode(cam_focus, OUTPUT); digitalWrite(cam_focus, LOW);
  pinMode(cam_shutter, OUTPUT); digitalWrite(cam_shutter, LOW);
  pinMode(batt_fet, OUTPUT); digitalWrite(batt_fet, LOW);
  pinMode(sd_ss, OUTPUT);

  /* START MCP23017 AND SET PINMODES AND PULLUPS */
  mcp.begin(); // Using default address 0

  mcp.pinMode(step_bwd, INPUT); mcp.pullUp(step_bwd, HIGH);
  mcp.pinMode(step_fwd, INPUT); mcp.pullUp(step_fwd, HIGH);
  mcp.pinMode(pwr_stat, INPUT); mcp.pullUp(pwr_stat, LOW);
  mcp.pinMode(switch_off_flag, INPUT);
  mcp.pinMode(switch_off, OUTPUT); mcp.digitalWrite(switch_off, HIGH);
  mcp.pinMode(limit_switch_front, INPUT); mcp.pullUp(limit_switch_front, HIGH);
  mcp.pinMode(limit_switch_back, INPUT); mcp.pullUp(limit_switch_back, HIGH);
  mcp.pinMode(bluetooth_pwr_toggle, OUTPUT); mcp.digitalWrite(bluetooth_pwr_toggle, HIGH);
  mcp.pinMode(connected_hardware, INPUT); mcp.pullUp(connected_hardware, HIGH);
  mcp.pinMode(dig_1, INPUT); mcp.pullUp(dig_1, HIGH);
  mcp.pinMode(btn_rotary, INPUT); mcp.pullUp(btn_rotary, HIGH);
  mcp.pinMode(ms1, OUTPUT); mcp.digitalWrite(ms1, HIGH);
  mcp.pinMode(ms2, OUTPUT); mcp.digitalWrite(ms2, HIGH);
  mcp.pinMode(ms3, OUTPUT); mcp.digitalWrite(ms2, HIGH);
  mcp.pinMode(step_enable, OUTPUT);
  mcp.pinMode(step_sleep, OUTPUT); mcp.digitalWrite(step_sleep, HIGH);

  /* SET UP INTERRUPTS */
  attachInterrupt(0, mcpInterrupt, FALLING); // ATMega external interrupt 0
  attachInterrupt(1, cancelStack, FALLING); // ATMega external interrupt 1

  pciSetup(ENC_A); // ATMega pin change interrupt
  pciSetup(ENC_B); // ATMega pin change interrupt

  mcp.setupInterrupts(true, false, LOW); // MCP23017 interupts (trigger ATMega external interrupt 0)
  mcp.setupInterruptPin(switch_off_flag, FALLING);
  mcp.setupInterruptPin(pwr_stat, CHANGE); //Need to catch both changes from LOW to HIGH and HIGH to LOW

  /* FINAL PRE-START CHECKS AND SETUP */
  Serial.begin(9600); // Start serial (always initialise at 9600 for compatibility with OLED)
  getHardware(); // Get the type of the attached stacker
  battMonitor(); // Check the battery level
  loadSettings(); // Attempt to load settings from SD card  
}



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/



/* UPDATE THE SCREEN WHEN DATA CHANGES
*
* Clears the screen then re-outputs it
*
*/
void screenUpdate() { /* WIPE THE SCREEN, PRINT THE HEADER AND SET THE CURSOR POSITION */

  screen.clearScreen(); // Repaints the screen blank
  screenPrintBluetooth(); // Display an icon for current bluetooth connection state
  screenPrintPowerSource(); // Display the current power source in top right corner

  screen.drawBox(1, 17, 128, 1); // Draw a horizontal line to mark out the header section
  screen.setPrintPos(0, 2); // Set text position to beginning of content area
  idle = false; // Flag that the system was recently used

}



/* PRINT TEXT ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
* char_length => The length of the string in the char buffer
* text => The string to be centered and printed
* print_pos_y => The text line to print on
*
*/
void screenPrint(byte char_length, char* text, byte print_pos_y = 4) {

  // Calculate the start point on x axis for the number of characters in the string to print it centrally
  byte print_pos_x = (OLED_COLS - char_length) / 2;
  
  screen.setPrintPos(print_pos_x, print_pos_y); // Set the start point for printing
  
  byte offset_x = char_length % 2 != 0 ? 4 : 0; // If the string to print is an odd number in length, apply an offset to centralise it
  byte offset_y = print_pos_y == 4 ? -2 : 0; // If printing on line 4, nudge upwards a bit to prevent lower parts of characters like 'g' being cut off

  screen.setTextPosOffset(offset_x, offset_y); // Set the required positioning offsets defined above
  
  screen.print(text); // Finally, print the centered text to the screen
  
}



/* SET HARDWARE CONSTANT
*
* Detect the stacking hardware connected to and set constant
* The controller relies on the assumption that 1 full step made by the stepper motor will move the
* linear stage by 1 (or as close as possible to 1) micron.
* To achieve this relationship, the number of step signals generated should be multiplied by a constant
* (obtained through a little maths and experimentation) which is unique to your hardware.
*
*/
void getHardware() {
  if (mcp.digitalRead(connected_hardware) == HIGH) {
    active_hardware_calibration_setting = 1;
  } else {
    active_hardware_calibration_setting = 1.5; // TODO: This is a random example number, work out the actual one
  }
}



/* LOAD AND SAVE SETTINGS
*
* load settings from SD card/ save settings to SD card
*
*/
void loadSettings() {
  
  // If a connection to the SD card couldn't be established
  if (!SD.begin(sd_ss)) {
    screenUpdate();
    screenPrint(sprintf_P(char_buffer, PSTR("SD error")), char_buffer, 4);
    delay(1000);
    return;
  }

  // Otherwise attempt to get the settings string from the SD card
  settings_file = SD.exists("tmpdata.txt") ? SD.open("tmpdata.txt", FILE_READ) : SD.open("settings.txt", FILE_READ);
  
  // If there was an issue attempting to open the file
  if (!settings_file) {

    screenPrint(sprintf_P(char_buffer, PSTR("SD file error")), char_buffer, 4);
    delay(1000);
    screenPrint(sprintf_P(char_buffer, PSTR("Using defaults")), char_buffer, 4);
    delay(1000);
    
  // Otherwise start copying the settings from the file into memory  
  } else {
    
    byte setting_index = 0; // Which menu setting to populate
    byte property_index = 0; // Index of the menu setting property to populate next
    byte char_buffer_index = 0; // Index of the char buffer to add next character from the sd read buffer
    char file_character; // The character currently in the sd read buffer
    boolean record_chars = false; // Whether to copy or ignore incoming characters

    while (settings_file.available()) { // Until the end of the file is reached, read it one character at a time

      file_character = settings_file.read(); // Current character in the file

      // Don't bother trying to process anything on the settings line until reaching an '{' 
      // The preceeding content is to make the file human readable and not used by the controller
      if (record_chars) { // If in a section of the file containing settings data

        // Example of a settings line from file:
        //
        // Slice size = {15,10,250,1};
        //
        // A ',' marks the end of a setting property and a '}' marks the end of the parent setting
        // If the current character is neither of these then it's part of, or the whole of a setting property and should be processed
        if (file_character != ',' && file_character != '}') {
          char_buffer[char_buffer_index] = file_character; // Add character to char buffer
          char_buffer_index++; // Advance to next index of char buffer to store any subsequent character for this setting property
          char_buffer[char_buffer_index] = '\0'; // Null terminate the string 
        }
        //settings_file.readBytesUntil(',' || '}', char_buffer, 4);


        // When all the characters for the current setting property have been captured, copy the char buffer contents to the settings struct in memory
        if (file_character == ',' || file_character == '}') {
          switch (property_index) { // The index of the current setting property
            case 0: // Setting value
              settings[setting_index].value = atoi(char_buffer);
              break;
            case 1: // Setting value lower limit
              settings[setting_index].lower = atoi(char_buffer);
              break;
            case 2: // Setting value upper limit
              settings[setting_index].upper = atoi(char_buffer);
              break;
            case 3: // Any multiplier that functions should apply to the setting value
              settings[setting_index].multiplier = atoi(char_buffer);
              break;
          }

          memset(char_buffer, 0, sizeof(char_buffer)); // Clear the char buffer for the next property
          char_buffer_index = 0; // Reset the char buffer indexer
          property_index++; // Move to the next property of the current setting

          // Once all properties of a setting have been copied
          if (file_character == '}') { 
            record_chars = false; // Stop recording from the sd read buffer
            property_index = 0; // Reset the setting property indexer
            setting_index++; // And move to the next setting
          }

        }
      }
      // Start recording from the next recieved character onwards
      if(file_character == '{'){
        record_chars = true;
      }
    }
    
    settings_file.close(); // Close the file once it has been read
    SD.remove("tmpdata.txt"); // Delete the temporary settings file if it exists  
    
  }

}



void saveSettings(boolean sleep = false) {

  //Open and empty the settings file if it exists, otherwise create it
  settings_file = sleep == true ? SD.open("tmpdata.txt", O_WRITE | O_CREAT | O_TRUNC) : SD.open("settings.txt", O_WRITE | O_CREAT | O_TRUNC);
  
  screenUpdate();
  screenPrint(sprintf_P(char_buffer, PSTR("Saving settings")), char_buffer, 2);

  // Loop through settings in memory and write back to the file
  for (byte i = 0; i < settings_count; i++) {
    
    stringConstants thisSettingTitle; //Retrieve the setting title from progmem
    memcpy_P (&thisSettingTitle, &settings_titles[i], sizeof thisSettingTitle);
    settings_file.write(thisSettingTitle.title);    
    settings_file.write(" = {");
    settings_file.write(settings[i].value);
    settings_file.write(",");
    settings_file.write(settings[i].lower);
    settings_file.write(",");
    settings_file.write(settings[i].upper);
    settings_file.write(",");
    settings_file.write(settings[i].multiplier);
    settings_file.write("};");
    settings_file.println();
        
  }

  settings_file.close(); // Save the file

}



/* POWER OFF
*
* The controller powers itself off by pulling the enable pin on the voltage regulator low
*
*/
void sysOff() {

  screenUpdate();
  screenPrint(sprintf_P(char_buffer, PSTR("System off")), char_buffer, 2);
  delay(1000);
  mcp.digitalWrite(switch_off, LOW);

}



/* BATTERY VOLTAGE READING
*
* Uses a mosfet to connect an analogue pin to the battery through a voltage divider
* Use the exact values of resistors and VCC as measured with a multimeter for most accuracy
* 12v Max input for battery
*
*/
void battMonitor() {

  if (mcp.digitalRead(pwr_stat) == HIGH) { // If currently running on battery

    float resistor_1 = 3.28; // Exact value of 3.3k resistor used
    float resistor_2 = 10.03; // Exact value of 10k resistor used
    float vcc = 3.25; // Exact value of 3.3v VCC

    digitalWrite(batt_fet, HIGH); // Connect the battery voltage line for reading
    calc_volts = (analogRead(batt_sense) * vcc) / 1024.00; // Get an analogue reading of the battery voltage measurement line
    calc_volts /= resistor_1 / (resistor_1 + resistor_2); // Factor in the voltage divider
    digitalWrite(batt_fet, LOW); // Disconnect the battery voltage line again

  }
}



/* BLUETOOTH PORT POWER
*
* Requires a compatible bluetooth breakout board (e.g. HC-05) connected to header H_1
* When enabled the VCC pin on H_1 is switched on via mosfet to power the breakout board
*
*/
void blueToothPower() {

  static byte last_bt_status;

  if (!start_stack && (last_bt_status != settings[10].value)) {
    mcp.digitalWrite(bluetooth_pwr_toggle, settings[10].value == 1 ? LOW : HIGH);
  }

  last_bt_status = settings[10].value;

}



/* ENABLE OR DISABLE THE STEPPER DRIVER AND SET DIRECTION
*
* enable => Enables the stepper driver if true, Disables if false and option set
* direction => Set as 1 for forward direction, 0 for backward direction
* toggle_direction => Toggle the direction to be the opposite of its current state
*
*/
void stepperDriverEnable(boolean enable = true, byte direction = 1, boolean toggle_direction = false) {

  if (enable) {
    mcp.digitalWrite(step_enable, LOW); // Enable the stepper driver
    
    if (toggle_direction) { // Toggle the direction
      direction = previous_direction;
    }
    
    digitalWrite(step_dir, direction); // Set the direction
    stepperDriverMicroStepping(); // Set the degree of microstepping
    previous_direction = direction; // Set the new direction for future toggle_direction calls
  } 
  
  else if (can_disable_stepper) {
    mcp.digitalWrite(step_enable, HIGH);
  }

}



/* MOVE STAGE FORWARD BY ONE SLICE */
void stepperMoveOneSlice(byte direction = 1){

  byte slice_size = settings[1].value;
  byte hardware = active_hardware_calibration_setting;
  byte unit_of_measure = uom_multipliers[settings[7].value];
  byte micro_steps = micro_stepping[0][settings[9].value][0];

  unsigned int rounded_steps = (slice_size * hardware * unit_of_measure * micro_steps) - 0.5;
  Serial.print(rounded_steps);
  
  stepperDriverEnable(true, direction, false);

  for (unsigned int i = 0; i < rounded_steps; i++) {

    stepperDriverStep(); // Send a step signal to the stepper driver
    if (!stepperDriverInBounds() || stackCancelled()) break; // Exit early if the stack has been cancelled or a limit switch is hit

  }
  
  stepperDriverEnable(false);

}


/* MOVE STAGE BACKWARD AND FORWARD */
void stepperDriverManualControl(byte direction = 1, boolean serial_control = false, boolean short_press = false) {

    if ((mcp.digitalRead(0) == LOW || mcp.digitalRead(1) == LOW || serial_control == true) && stepperDriverInBounds()) {
      
      unsigned long button_down = millis();
      char* manual_ctl_strings[2] = {"<", ">"};
      
      screenUpdate();
      screenPrint(sprintf_P(char_buffer, PSTR("Moving stage")), char_buffer, 2);
      screen.setPrintPos(4, 4);
      for (byte i = 0; i < 8; i++) {
        screen.print(manual_ctl_strings[direction]);
      }

      stepperDriverEnable(true, direction); // Enable the stepper driver and set the direction
      
      if(short_press){ // Move the stage by one focus slice - useful for adding extra slices to the front and end of a stack
        stepperMoveOneSlice(1);
      } else { // Move the stage for as long as the control button is pressed
        while ((mcp.digitalRead(direction) == LOW || serial_control == true) && stepperDriverInBounds() && Serial.available() == 0) {
          stepperDriverStep();
          if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
        }
      }

    }

    stepperDriverEnable(false); // Disable the stepper driver
    update_display = true; // Go back to displaying the active menu item once manual control button is no longer pressed

}



/* APPLICATION CONNECTION STATUS
*
* NOT CURRENTLY USED
*
* Sends a keep-alive token and expects one back the next time it is called
* Otherwise assumes the connection with the remote application has been lost
* Also sets the icon for current connection status
*
*/
void appConnection() {

  static boolean last_app_conn_stat;

  //app_conn_icon = app_connected == true ? : ;

  if (app_connected != last_app_conn_stat) {
    update_display = true;
  }

  last_app_conn_stat = app_connected;

  //Serial.print(F("k")); // Send token

}



/* SIGNAL CAMERA TO TAKE IMAGE(S)
*
* Optionally sends multiple delay seperated signals for exposure bracketing each focus slice
*
*/
void captureImages() {
  
  screenUpdate();

  for (byte i = 1; i <= settings[5].value; i++) { // Number of brackets to take per focus slice

    screenPrintPositionInStack();

    if (settings[5].value > 1) { // If more than one image is being taken, display the current position in the bracket
      screenPrint(sprintf_P(char_buffer, PSTR("Bracket %d/%d"), i, settings[5].value), char_buffer, 4);
    }
    
    pause(1500); // Allow vibrations to settle
    shutter(); // Take the image
    
    for (byte i = 0; i < settings[3].value; i++) { // Count down the pause for camera on the screen

      screenPrintPositionInStack();
      screenPrint(sprintf_P(char_buffer, PSTR("Resume in %ds"), settings[3].value - i), char_buffer, 4);
      pause(1000);
      
      if (stackCancelled()) break; // Exit early if the stack has been cancelled

    }

    if (stackCancelled()) break; // Exit early if the stack has been cancelled

  }
  
  update_display = true;
    
}



/* TRIGGER THE CAMERA SHUTTER
*
* Optionally sends two delay seperated signals to support mirror lockup
*
*/
void shutter() {

  for (byte i = 0; i <= settings[4].value; i++) {

    screenPrintPositionInStack();
    
    if (settings[4].value && i == 0) { // If mirror lockup enabled
      screenPrint(sprintf_P(char_buffer, PSTR("Mirror up")), char_buffer, 4);
    } else {
      screenPrint(sprintf_P(char_buffer, PSTR("Shutter")), char_buffer, 4);
    }
    
    pause(500);

    digitalWrite(cam_focus, HIGH); // Trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(cam_shutter, HIGH); // Trigger camera shutter

    pause(200); //Small delay needed for camera to process above signals

    digitalWrite(cam_shutter, LOW); // Switch off camera trigger signal
    digitalWrite(cam_focus, LOW); // Switch off camera focus signal

    if (settings[4].value && i == 0) { // If mirror lockup enabled
      pause(2000); // Pause between mirror up and shutter actuation
    }
  }
}



/* MAIN BUTTON
*
* Starts or stops a focus stack
*
*/
void btnPress(byte btn_pin) {

  btn_reading = btn_pin == 3 ? digitalRead(btn_pin) : mcp.digitalRead(btn_pin);
  

  if ((btn_reading == LOW) && (btn_previous == HIGH) && (millis() - btn_time > 250)) {
    
    boolean short_press = false;
    unsigned long btn_down_time = millis();
    
    while(millis() < btn_down_time + 500){ // Press and hold to start a stack
      if(btn_pin == 3 ? digitalRead(btn_pin) : mcp.digitalRead(btn_pin) == HIGH){ // Press and immedeately release to take test images
        short_press = true;
        break; 
      } 
    }
        
    btn_time = millis();  
  
    switch(btn_pin){
      case 0: case 1:
        short_press == true ? stepperMoveOneSlice(btn_pin) : stepperDriverManualControl(btn_pin);
        break;
      case 3:
        short_press == true ? captureImages() : startStack();
        break;
      case 10:
        short_press == true ? menuNav() : saveSettings();
    }
        
  }

  btn_previous = btn_reading;

}

void startStack() {

  start_stack = start_stack == true ? false : true;

}

// This is designed to flag a running stack as cancelled
// It's set with an interrupt with an emphasis on speed of execution
// So there's no debouncing or toggling of state
void cancelStack(){
  
  if(start_stack){
    start_stack = false; 
  }
  
}

void menuNav() {

  if (menu_item != 0) {
    traverse_menus = traverse_menus == true ? false : true;
  }

  update_display = true;

}



/* SYSTEM TASKS
*
* Switch Bluetooth on or off
* Check connection status of external applications
* Periodically check the battery level
* Switch off the controller if it has been unused for n minutes
*
*/
void sysTasks(boolean reset = false) {

  static byte seconds = 0; // Number of seconds elapsed
  static unsigned long one_second = millis() + 1000; // 1 second from now
  static byte minutes = 0; // Number of minutes elapsed
  static boolean conn_stat_polled = true;

  if(reset){
    seconds = 0;
    minutes = 0;
  }
  
  blueToothPower(); // Switch Bluetooth port on or off

  if (millis() >= one_second) { // Count each second
    seconds++;
    one_second = millis() + 1000;
    conn_stat_polled = false;
  }

  if (seconds % 5 == 0) { // Every 5 seconds
    if (!conn_stat_polled) {
      appConnection();
      conn_stat_polled = true;
    }
  }

  if (seconds == 60) {
    seconds = 0;
    minutes++;
    battMonitor();
  }

  // If enabled after n minutes switch off the controller
  if (sys_off && (minutes >= sys_off)) {
    //saveSettings(true); // Save current settings to a temp file to resume when controller is switched on again
    //sysOff();
  }

}



/* RETURN ANY CHANGE IN ROTARY ENCODER POSITION
*
* Using work by:
* http://www.circuitsathome.com/mcu/rotary-encoder-interrupt-service-routine-for-avr-micros
*
*/
int8_t encoderRead() {

  static int8_t enc_states[] = {
    //0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0 // Use this line instead to increment encoder in the opposite direction
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
  };

  static uint8_t old_AB = 0;

  old_AB <<= 2; // Remember previous state
  old_AB |= ( ENC_PORT & 0x03 ); // Add current state

  int8_t encoder_data = ( enc_states[( 0x0f & old_AB )]);

  static int8_t encoder_pulse_counter = 0; // Counts pulses from encoder

  if (encoder_data) { // If Gray Code returns a valid input (1 or -1)
    encoder_data == 1 ? encoder_pulse_counter++ : encoder_pulse_counter--;
  }

  if (encoder_pulse_counter > 3) { // Record the encoder was moved by one click (detent)
    increments++;
    encoder_pulse_counter = 0;
  }

  else if (encoder_pulse_counter < -3) { // Record the encoder was moved by one click (detent) in the opposite direction
    increments--;
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
ISR (PCINT1_vect) {

  encoderRead();

}



/* FLAG NEW INTERRUPTS FROM THE MCP23017 */
void mcpInterrupt() {

  detachInterrupt(0); // Detach the interrupt while the current instance is dealt with
  mcp_int_fired = true;
  idle = false;

}



/* PROCESS FLAGGED INTERRUPTS FROM THE MCP23017 */
void handleMcpInterrupt() {

  byte pin = mcp.getLastInterruptPin(); // Find out which pin on the mcp created the interrupt
  byte val = mcp.getLastInterruptPinValue(); // And get its current state

  if (pin == switch_off_flag) { // If a switchoff signal was recieved from the pushbutton controller
    sysOff();
  }
  
  if (pin == pwr_stat) { // If a manual stage control button was pressed
    battMonitor();
    update_display = true;
  }

  EIFR = 0x01; // Clear interrupts
  mcp_int_fired = false; // Reset the flag
  attachInterrupt(0, mcpInterrupt, LOW); // Re-attach the interrupt

}



/* PAUSE
*
* Delays are needed throughout the code, but these should be non-blocking
*
*/
void pause(unsigned int length) {

  unsigned long initial_millis = millis(); // Get the current millis
  unsigned long end_millis = initial_millis + length; // Define when the pause ends

  while (end_millis > millis()) { // If the end of the pause time hasn't yet been reached

    if (stackCancelled()) break; // Check continuously for any reason to end it early

  }

}



/* PRINT MENU NAVIGATION ARROWS ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintMenuArrows() {

  if (!start_stack) {
    if (traverse_menus) { // Move to the next menu item
      //TODO: CREATE ICON FOR FONT FILE TO REPLACE BELOW
      screen.drawBox(5, 52, 1, 9); // Left outward arrow
      screen.drawBox(4, 53, 1, 7);
      screen.drawBox(3, 54, 1, 5);
      screen.drawBox(2, 55, 1, 3);
      screen.drawBox(1, 56, 1, 1);

      screen.drawBox(122, 52, 1, 9); // Right outward arrow
      screen.drawBox(123, 53, 1, 7);
      screen.drawBox(124, 54, 1, 5);
      screen.drawBox(125, 55, 1, 3);
      screen.drawBox(126, 56, 1, 1);
    } else { // Change the value of the current menu item
      //TODO: CREATE ICON FOR FONT FILE TO REPLACE BELOW
      screen.drawBox(1, 52, 1, 9); // Left inward arrow
      screen.drawBox(2, 53, 1, 7);
      screen.drawBox(3, 54, 1, 5);
      screen.drawBox(4, 55, 1, 3);
      screen.drawBox(5, 56, 1, 1);

      screen.drawBox(122, 56, 1, 1); // Right inward arrow
      screen.drawBox(123, 55, 1, 3);
      screen.drawBox(124, 54, 1, 5);
      screen.drawBox(125, 53, 1, 7);
      screen.drawBox(126, 52, 1, 9);
    }
  }
}



/* PRINT BLUETOOTH ICON ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintBluetooth() {

  if (settings[10].value) { //if the bluetooth header (H_1) is active
    screen.drawBox(1, 2, 1, 1);
    screen.drawBox(1, 8, 1, 1);
    screen.drawBox(2, 3, 1, 1);
    screen.drawBox(2, 7, 1, 1);
    screen.drawBox(3, 4, 1, 1);
    screen.drawBox(3, 6, 1, 1);
    screen.drawBox(4, 1, 1, 9);
    screen.drawBox(5, 1, 1, 1);
    screen.drawBox(5, 5, 1, 1);
    screen.drawBox(5, 9, 1, 1);
    screen.drawBox(6, 2, 1, 1);
    screen.drawBox(6, 4, 1, 1);
    screen.drawBox(6, 6, 1, 1);
    screen.drawBox(6, 8, 1, 1);
    screen.drawBox(7, 3, 1, 1);
    screen.drawBox(7, 7, 1, 1);
  }

}



/* PRINT APPLICATION CONNECTION ICON ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintConnection() {

  screen.print(app_connected == true ? F("AC") : F("AU"));

}



/* PRINT THE ACTIVE POWER SOURCE ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintPowerSource() {

  if (mcp.digitalRead(pwr_stat) == LOW) { // Draw plug symbol (dc adapter connected)
    screen.drawBox(115, 1, 3, 1);
    screen.drawBox(112, 2, 7, 1);
    screen.drawBox(115, 3, 13, 2);
    screen.drawBox(112, 5, 7, 1);
    screen.drawBox(115, 6, 3, 1);
  } else { // Draw battery symbol and print current voltage
    screen.setPrintPos(7, 0);
    screen.print(calc_volts);
    screen.print(F("v"));
    screen.drawBox(112, 3, 2, 2);
    screen.drawBox(114, 1, 14, 6);
  }

}



/* PRINT THE CURRENT POSITION IN A RUNNING FOCUS STACK ON DIGOLE OLED SCREEN
*
* Assumes the default font is in use (allowing 4 rows of 16 characters)
*
*/
void screenPrintPositionInStack() {

  screenUpdate();
  if(slice_count > 0){ // Default behaviour in a running stack
    screenPrint(sprintf_P(char_buffer, PSTR("Slice %d/%d"), slice_count, settings[2].value), char_buffer, 2);
  } else { // If called by a function when in setup
    screenPrint(sprintf_P(char_buffer, PSTR("Single slice")), char_buffer, 2);
  }

}



/* CONTROL VIA SERIAL
*
* Checks for incoming characters over serial and runs any mtaching functions
* Allows for remote control via USB or Bluetooth
*
*/
void serialCommunications() {

  if (Serial.available() > 0) {

    idle = false;
    int serial_in = Serial.read(); // read the incoming byte:

    switch (serial_in) {

      case 'a': // Start or stop stack
        startStack();
        break;

      case 'b': // Toggle menu navigation
        menuNav();
        break;

      case 'c': // Increment active setting variable
        increments++;
        break;

      case 'd': // Decrement active setting variable
        increments--;
        break;

      case 'e': // Move stage backward by one slice
        stepperDriverManualControl(0, true, true);
        break;

      case 'f': // Move stage backward until another character is sent
        stepperDriverManualControl(0, true, false);
        break;

      case 'g': // Move stage forward by one slice
        stepperDriverManualControl(1, true, true);
        break;

      case 'h': // Move stage forward until another character is sent
        stepperDriverManualControl(1, true, false);
        break;

      case 'i': // Take a test image
        shutter();
        break;
        
      case 'j': // Take a suite of test images
        captureImages();
        break;

      case 'k': // Switch Bluetooth on or off
        settings[10].value = settings[10].value == 1 ? 0 : 1;
        update_display = true;
        break;

      case 'l':
        saveSettings();
        break;

      case 'm': // Switch off controller
        sysOff();
        break;

      case 'n': // Keep connection alive
        app_connected = true;
        break;

      default: // Unmatched character - pretend the function call never happened
        idle = true;
    }
  }
}



/* CHECK IF THE FOCUS STACK HAS BEEN CANCELLED */
boolean stackCancelled() {
  
  if(slice_count){
    if (Serial.available() > 0) {
      int serial_in = Serial.read(); // Read the incoming byte:
      if(serial_in == 'a') startStack(); // Stop stack
    }
    
    if (!start_stack && time_stack_started && (millis() > time_stack_started + 2000)) {
      return true; // Stack is cancelled if has been running for over 2 seconds (debouncing interrupt without using a delay)
    }
  
    start_stack = true;
  }
  
  return false;

}



/* CLEAN UP AFTER FOCUS STACK COMPLETION
*
* Optionally return the stage to its starting position
* Reset various variables to inital values
*
*/
void stackEnd() {

  screenUpdate();
  screenPrint(sprintf_P(char_buffer, start_stack == false ? PSTR("Stack cancelled") : PSTR("Stack completed")), char_buffer, 2);
  
  // Calculate how many minutes and seconds the stack ran for before it completed or was cancelled
  unsigned int seconds = (millis() - time_stack_started) / 1000; // Number of seconds since stack was started
  
  div_t mins_secs = div (seconds, 60); // Number of minutes and seconds since stack was started
    
  screenPrint(sprintf_P(char_buffer, PSTR("%02dm:%02ds"), mins_secs.quot, mins_secs.rem), char_buffer, 4);

  delay(3000);

  if (settings[6].value == 1) { // If return stage to start position option is enabled

    stepperDriverEnable(true, 0); // Enable the stepper driver and set the direction to backwards

    screenPrint(sprintf_P(char_buffer, PSTR("Returning")), char_buffer, 4);
    pause(1000);

    for (int i; i < slice_count; i++) {

      stepperMoveOneSlice(1);

    }

    if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
    stepperDriverEnable(false); // Disable the stepper driver

  }

  menu_item = 0; // Set menu to first option screen
  home_screen = true; // Reinstate the home screen
  slice_count = 0; // Reset pic counter
  start_stack = false; // Return to menu options section
  update_display = true; // Write the first menu item to the screen

}



/* PULL STAGE AWAY FROM TRIPPED LIMIT SWITCH */
void stepperDriverClearLimitSwitch() {

  screenUpdate();
  
  screenPrint(sprintf_P(char_buffer, PSTR("Limit hit")), char_buffer, 2);
  screenPrint(sprintf_P(char_buffer, PSTR("Returning")), char_buffer, 4);

  stepperDriverEnable(true, 0, true); // Enable the stepper driver and toggle the direction

  while (mcp.digitalRead(limit_switch_front) == LOW || mcp.digitalRead(limit_switch_back) == LOW) { //turn stepper motor for as long as  the limit switch remains pressed

    stepperDriverStep();

  }

  stepperDriverEnable(true, 0, true); // Enable stepper driver and toggle the direction

  update_display = true;
  stepperDriverEnable(false); // Disable the stepper driver

  if (start_stack) { // If a stack was in progress, cancel it
    start_stack = false;
    stackCancelled();
  }

}



/* CHECK IF STAGE IS IN BOUNDS OF TRAVEL I.E. NEITHER LIMIT SWITCH IS TRIPPED */
boolean stepperDriverInBounds() {

  if (mcp.digitalRead(limit_switch_front) == HIGH && mcp.digitalRead(limit_switch_back) == HIGH) {
    return true;
  }

  return false;

}



/* SET DEGREE OF MICROSTEPPING FOR A4988 STEPPER DRIVER TO USE
*
* Loops through the active microstepping subarray and writes the A4988's MSx pins HIGH or LOW accordingly
* MSx pins are MCP23017 11 - 13
*
*/
void stepperDriverMicroStepping() {

  byte active_ms = settings[8].value;

  for (byte i = 0, j = 11; i < 2; i++, j++) {

    mcp.digitalWrite(j, micro_stepping[1][active_ms][i]);

  }

}



/* SEND STEP SIGNAL TO A4988 STEPPER DRIVER */
void stepperDriverStep() {

  digitalWrite(do_step, LOW); // This LOW to HIGH change is what creates the
  digitalWrite(do_step, HIGH); // "Rising Edge" so the driver knows when to step
  delayMicroseconds(settings[8].value * 1000); // Delay time between steps, too short and motor may stall or miss steps

}



/* CHANGE THE ACTIVE MENU SETTING'S VALUE
*
* May be implemented by either the rotary encoder or a command via serial
*
* setting_value => The variable to change the value of
* setting_lower => The lower bounds for constraining the value of var
* setting_upper => The upper bounds for constraining the value of var
* setting_multipler = Factor to multiply the change in value of var by
*
*/
void settingUpdate(int &setting_value, int setting_lower, int setting_upper, int setting_multiplier = 1) {

  setting_value = constrain(setting_value, setting_lower, setting_upper); // Keep variable value within specified range

  if (increments != 0) { // If the variable's value was changed at least once since the last check
    setting_value += (setting_multiplier * increments); // Add or subtract from variable
    increments = 0; // Reset for next check
    update_display = true;
  }

}



/* OUTPUT MENUS
*
* Publishes menus which refresh whenever the selected variable changes
*
*/
void menuInteractions() {

  char* repeated_strings[] = {"Enabled", "Disabled"};

  int lower_limit = home_screen == true ? 0 : 1; // The home screen is only appears once when the menu is first loaded, it's skipped when looping around the options

  if (traverse_menus) { // Move through menu items
    settingUpdate(menu_item, lower_limit - 1, settings_count); // Display the currently selected menu item
    if (menu_item == settings_count) menu_item = lower_limit; // Create looping navigation
    if (menu_item == lower_limit - 1) menu_item = settings_count - 1; // Create looping navigation
  } else { // Otherwise change the value of the current menu item
    settingUpdate(settings[menu_item].value, settings[menu_item].lower - 1, settings[menu_item].upper + 1, settings[menu_item].multiplier);
    if (settings[menu_item].value > settings[menu_item].upper) settings[menu_item].value = settings[menu_item].lower; // Create looping navigation
    if (settings[menu_item].value < settings[menu_item].lower) settings[menu_item].value = settings[menu_item].upper; // Create looping navigation
  }

  if (menu_item != 0) home_screen = false; // Remove homescreen from the menu loop once navigation begun

  if (update_display) { // Refresh menu content if the active variable has changed

    int menu_var = settings[menu_item].value;
    byte string_length;
    screenUpdate();
    screenPrintMenuArrows(); // Print menu arrows

    switch (menu_item) { // The menu options

      case 0:
        string_length = sprintf_P(char_buffer, PSTR("Setup"));
        break;

      case 1: // Change the number of increments to move each time
        string_length = sprintf(char_buffer, "%d", menu_var);
        break;

      case 2: // Change the number of slices to create in the stack
        string_length = sprintf(char_buffer, "%d", menu_var);
        break;

      case 3: // Change the number of seconds to wait for the camera to capture an image before continuing
        string_length = sprintf_P(char_buffer, PSTR("%ds"), menu_var);
        break;

      case 4: // Toggle mirror lockup for the camera
        string_length = menu_var == 1 ? sprintf(char_buffer, repeated_strings[0]) : sprintf(char_buffer, repeated_strings[1]);
        break;

      case 5: // Change the number of images to take per focus slice (exposure bracketing)
        string_length = menu_var == 1 ? sprintf(char_buffer, "%s", repeated_strings[1]) : sprintf(char_buffer, "%d", menu_var);
        break;

      case 6: // Toggle whether camera/subject is returned the starting position at the end of the stack
        string_length = menu_var == 1 ? sprintf(char_buffer, repeated_strings[0]) : sprintf(char_buffer, repeated_strings[1]);
        break;

      case 7: // Select the unit of measure to use for focus slices: Microns, Millimimeters or Centimeters
        string_length = sprintf(char_buffer, "%c%c", uom_chars[menu_var], uom_chars[1]);
        break;

      case 8: // Adjust the stepper motor speed (delay in microseconds between slice_size)
        // A smaller number gives faster motor speed but reduces torque
        // Setting this too low may cause the motor to miss steps or stall
        string_length = sprintf_P(char_buffer, PSTR("%d000uS"), menu_var);
        break;

      case 9: // Adjust the degree of microstepping made by the a4988 stepper driver
        // More microsteps give the best stepping resolution but may require more power for consistency and accuracy
        string_length = sprintf_P(char_buffer, PSTR("1/%d"), micro_stepping[0][menu_var][0]);
        break;

      case 10: // Toggle power to an external 3.3v bluetooth board e.g. HC-05
        menu_var = !menu_var;
        string_length = sprintf(char_buffer, repeated_strings[menu_var]);
        break;

    }

    stringConstants flashString; //Retrieve the setting title from progmem
    memcpy_P(&flashString, &settings_titles[menu_item], sizeof flashString);
    
    screenPrint(strlen(flashString.title), flashString.title, 2); // Print the menu setting title

    if (!traverse_menus) { // Invert the colour of the current menu item to indicate it is editable
      byte boxWidth = (string_length * 8) + 2;
      byte leftPos = ((128 - boxWidth) / 2);
      screen.setMode('~');
      screen.drawBox(leftPos, 48, boxWidth, 22);
      screenPrint(string_length, char_buffer, 4); // Print the menu setting value
    } else {
      screenPrint(string_length, char_buffer, 4); // Print the menu setting value
    }
    update_display = false;

  }

}



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
//  MAIN LOOP                                                                                           //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/
void loop() {



  /* MENUS AND STAGE CONTROL
  *
  * Manual control of linear stage is only enabled when in this section of the loop
  * - prevents running stacks being ruined by accidental button presses
  *
  * Menu navigation controlled either by rotary encoder with integral push button, or via serial
  *
  */
  if (!start_stack) { // User settings menus and manual stage control

    if (mcp_int_fired) handleMcpInterrupt(); // Catch interrupts and run any required functions
    btnPress(3);
    btnPress(10);
    btnPress(0);
    btnPress(1);
    serialCommunications();  // Check for commands via serial
    menuInteractions(); // Change menu options and update the screen when changed
    sysTasks(); // Run timer driven tasks such as battery level checks and auto switch off

  }



  /* FOCUS STACK
  *
  * Images are captured, the stage is advanced and the process is repeated for as many times as needed
  *
  */
  else {

    byte slice_size = settings[1].value;
    byte slices = settings[2].value;
    
    time_stack_started = millis();

    slice_count = 1; // Register the first image(s) of the stack is being taken
    screenPrintPositionInStack(); // Print the current position in the stack
    captureImages(); // Take the image(s) for the first slice of the stack

    for (unsigned int i = 1; i < slices; i++) { // For each subsequent focus slice in the stack

      slice_count++; // Record slices made in the stack so far

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      screenPrintPositionInStack(); // Print the current position in the stack
      // Print that the stage is being advanced by x units
      screenPrint(sprintf_P(char_buffer, PSTR("Advance %d%c%c"), slice_size, uom_chars[settings[7].value], uom_chars[1]), char_buffer, 4);

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(true, 1); // Enable the stepper driver and set the direction to forwards
      stepperMoveOneSlice(1); // Move forward by one focus slice

      if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch(); // Clear hit limit switch
      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(false); // Disable the stepper driver
      captureImages(); // Take image(s)

    }

    stackEnd(); // End of focus stack tasks

  }
}
