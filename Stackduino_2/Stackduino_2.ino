/*////////////////////////////////////////////////////////////////////////////////////////////////////////
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
#define ENC_A A0 // Rotary encoder
#define ENC_B A1 // Rotary encoder
#define ENC_PORT PINC // Rotary encoder

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
  char title [16];
} stringConstants;

const stringConstants settings_titles[] PROGMEM = { // A struct of char arrays stored in Flash
  {"Stackduino v2.2"},
  {"Slice size"},
  {"Slices"},
  {"Pause time"},
  {"Mirror lockup"},
  {"Bracketing"},
  {"Return to home"},
  {"Units"},
  {"Stepper speed"},
  {"Microstepping"},
  {"Bluetooth"}
};

char uom_chars[3] = {'u', 'm', 'c'};

struct Settings { // A struct type for storing settings

  int value; // The setting value
  int lower; // The lowest value the setting may have
  int upper; // The highest value the setting may have
  int multiplier; // Any multiplier to apply when the setting is incremented

} settings[] = { // A struct of user menu settings
  {}, // The home screen - not actually used by any functions but the placeholder array must exist
  {10, 10, 250, 1}, // "Slice size"
  {10, 10, 250, 10}, // "Number of slices"
  {5, 1, 60, 1}, // "Pause time"
  {0, 0, 1, 1}, // "Mirror lockup"
  {1, 1, 10, 1}, // "Bracketing"
  {0, 0, 1, 1}, // "Return to start"
  {0, 0, 2, 1}, // "Unit of measure"
  {2, 1, 8, 1}, // "Stepper speed"
  {4, 0, 4, 1}, // "Microstepping"
  {1, 0, 1, 1} // "Bluetooth"
};

byte settings_count = sizeof(settings) / sizeof(Settings); // The number of settings



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
volatile byte btn_main_reading, btn_rotary_reading; // The current reading from the button
volatile byte btn_main_previous = LOW, btn_rotary_previous = LOW; // The previous reading from the button
volatile long btn_main_time = 0, btn_rotary_time = 0; // The last time the button was toggled

boolean app_connected = false; // Whether a remote application is actively communicating with the controller
boolean step_disabled = true; // Whether to disable the A4988 stepper driver when possible to save power and heat
boolean idle = true; // True until the controller is interacted with
boolean update = true; // True whenever functions are called which change a setting's value
boolean update_header = true; // True whenever the value of an item in the header changes
boolean home_screen = true;
boolean previous_direction;

float calc_volts = 0.00; // The actual battery voltage as calculated through the divider

int menu_item = 0; // The active menu item
int increments = 0; // Count increments the active menu setting should be changed by on the next poll
int serial_in; // Store latest byte from incoming serial stream

byte sys_off = 20; // Minutes of inactivity to wait until controller switches itself off - set to 0 to disable
byte slice_count = 0; // Count of number of focus slices made so far in the stack

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
  attachInterrupt(1, btnMain, FALLING); // ATMega external interrupt 1

  pciSetup(ENC_A); // ATMega pin change interrupt
  pciSetup(ENC_B); // ATMegapin change interrupt

  mcp.setupInterrupts(true, false, LOW); // MCP23017 interupts (trigger ATMega external interrupt 0)
  mcp.setupInterruptPin(step_bwd, FALLING);
  mcp.setupInterruptPin(step_fwd, FALLING);
  mcp.setupInterruptPin(switch_off_flag, FALLING);
  mcp.setupInterruptPin(btn_rotary, FALLING);

  /* FINAL PRE-START CHECKS AND SETUP */
  Serial.begin(9600); // Start serial (always initialise at 9600 for compatibility with OLED)
  battMonitor(); // Check the battery level
  getHardware(); // Get the type of the attached stacker
  //saveSettings();
  loadSettings(); // Attempt to load settings from SD card

  stepperDriverClearLimitSwitch(); // Make sure neither limit switch is hit on startup - rectify if so
}



/*////////////////////////////////////////////////////////////////////////////////////////////////////////
// FUNCTIONS                                                                                            //
////////////////////////////////////////////////////////////////////////////////////////////////////////*/



/* UPDATE THE SCREEN WHEN DATA CHANGES
*
* Clears the screen then re-outputs it
*
* print_pos_y => Which text line to set the initial print position on
* print_pos_x => Which character on the text line to set the initial print position on
* text => Optional text string to print
*
*/
void screenUpdate() { /* WIPE THE SCREEN, PRINT THE HEADER AND SET THE CURSOR POSITION */

  screen.clearScreen();
  screenPrintBluetooth(); // Display an icon for current bluetooth connection state
  screenPrintPowerSource(); // Display the current power source in top right corner
  screen.drawBox(1, 15, 128, 1); // Draw a horizontal line to mark out a header section
  screen.setPrintPos(0, 2); // Set text position to beginning of content area
  idle = false; // Flag that the system was recently used

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
void screenPrintCentre(char* text, byte string_length, byte print_pos_y = 4) {

  byte offset = (16 - string_length) / 2; // Calculate the offset for the number of characters in the string

  screen.setPrintPos(offset, print_pos_y);

  if (string_length % 2 != 0) { // Dividing an odd numbered int by 2 discards the remainder, creating an offset which is half a text character width too short
    screen.setTextPosOffset(4, -2); // So the text needs to be nudged to the right a bit on a pixel level to centre it properly
  }

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
  screenUpdate();
  screen.print(F(" Hardware mode: "));
  screen.setPrintPos(0, 4);
  if (mcp.digitalRead(connected_hardware) == HIGH) {
    active_hardware_calibration_setting = 1;
    screen.print(F("     Studio     "));
  } else {
    active_hardware_calibration_setting = 1.5; // TODO: This is a random example number, work out the actual one
    screen.print(F("    Portable    "));
  }
  delay(2500);
}



/* LOAD AND SAVE SETTINGS
*
* load settings from SD card/ save settings to SD card
* Stored in JSON format
*
*/
void loadSettings() {

  screenUpdate();
  screen.print(F("Loading settings"));

  if (!SD.begin(sd_ss)) {
    screen.setPrintPos(0, 4);
    screen.print(F(" SD card error! "));
    delay(2000);
    screen.setPrintPos(0, 4);
    screen.print(F(" Using defaults "));
    delay(2000);
    return;
  }

  // Get the settings string from the SD card
  settings_file = SD.open("settings.txt", FILE_READ);
  if (settings_file) {

    byte setting_index = 0; // Index of the menu setting to populate
    byte property_index = 0; // Index of the menu setting property to populate
    char file_string[4]; // A string container to fill with characters from the sd read buffer
    byte file_string_index = 0; // Position in string container to add next character from the sd read buffer
    char file_character; // The character currently in the sd read buffer
    boolean start_recording = false; // Whether to store or ignore incoming characters
    byte loading = 0;

    while (settings_file.available()) { // Until the end of the file is reached

      file_character = settings_file.read(); // Current character in the file

      // Don't bother trying to process anything on the settings line until reaching an opening brace
      // The preceeding content is just to make the file human readable
      if (start_recording) { // If in a section of the file containing settings data

        // A ',' marks the end of a setting sub-item and a '}' marks the end of the parent setting
        // If the current character is neither of these then it's part of, or the whole of a setting property and should be processed
        if (file_character != ',' && file_character != '}') {
          file_string[file_string_index] = file_character; // Add character to temporary string
          file_string_index++; // Advance to next index of temporary string to store any subsequent character for this seting property
          file_string[file_string_index] = '\0'; // Null terminate the string 
        }

        // When all the characters for the current setting property have been captured, copy the temporary string contents to the settings struct in memory
        if (file_character == ',' || file_character == '}') {
          switch (property_index) { // The index of the current setting sub-item
            case 0:
              settings[setting_index].value = atoi(file_string);
              break;
            case 1:
              settings[setting_index].lower = atoi(file_string);
              break;
            case 2:
              settings[setting_index].upper = atoi(file_string);
              break;
            case 3:
              settings[setting_index].multiplier = atoi(file_string);
              break;
          }

          if (file_character == '}') { // If at the end of a settings line
            file_string[0] = '\0'; // Clear the temporary string array
            file_string_index = 0; // Reset the file string indexer
            property_index = 0; // Reset the setting property indexer
            start_recording = false; // And stop recording from the sd read buffer
            setting_index++; // Move to the next setting
          } else {
            property_index++; // Otherwise move to the next property of the current setting
          }

        }
      }
      // Start recording from the next recieved character onwards
      if(file_character == '{'){
        start_recording = true;
      }
    }
    
    settings_file.close(); // Close the file:
    
  } else {
    // if the file didn't open, print an error:
    screen.setPrintPos(0, 4);
    screen.print(F(" SD file error! "));
    delay(2000);
    screen.setPrintPos(0, 4);
    screen.print(F(" Using defaults "));
    delay(2000);
  }

}



void saveSettings() {

  //Open and empty the settings file if it exists, otherwise create it
  settings_file = SD.open("settings.txt", O_WRITE | O_CREAT | O_TRUNC);

  char progmem_buffer[16 + 1]; // make sure this is large enough for the largest string it must hold

  // Loop through settings and write to file
  for (byte i = 0; i < settings_count; i++) {
    stringConstants thisSettingTitle; //Retrieve the setting title from progmem
    memcpy_P (&thisSettingTitle, &settings_titles[i], sizeof thisSettingTitle);
    settings_file.write(thisSettingTitle.title);
    Serial.print(thisSettingTitle.title);
    settings_file.write(" = {");
    Serial.print(" = {");
    settings_file.write(settings[i].value);
    Serial.print(settings[i].value);
    settings_file.write(",");
    Serial.print(",");
    settings_file.write(settings[i].lower);
    Serial.print(settings[i].lower);
    settings_file.write(",");
    Serial.print(",");
    settings_file.write(settings[i].upper);
    Serial.print(settings[i].upper);
    settings_file.write(",");
    Serial.print(",");
    settings_file.write(settings[i].multiplier);
    Serial.print(settings[i].multiplier);
    settings_file.write("};");
    Serial.print("};");
    settings_file.println();
    Serial.println();
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
  //saveSettings();
  screen.print(F("   System off   "));
  delay(2000);
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



/* APPLICATION CONNECTION STATUS
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
    update = true;
  }

  last_app_conn_stat = app_connected;

  //Serial.print(F("k")); // Send token

}



/* MAIN BUTTON
*
* Starts or stops a focus stack
*
*/
void btnMain() {

  byte btn_debounce = 250;
  btn_main_reading = digitalRead(btn_main);

  if (btn_main_reading == LOW && btn_main_previous == HIGH && millis() - btn_main_time > btn_debounce) {
    startStack();
    btn_main_time = millis();
  }

  btn_main_previous = btn_main_reading;

}

void startStack() {

  start_stack = start_stack == true ? false : true;

}

/* ROTARY ENCODER BUTTON
*
* Toggles between menu navigation and variable editing
*
*/
void btnRotary() {

  byte btn_debounce = 250;
  btn_rotary_reading = mcp.digitalRead(btn_rotary);

  if (btn_rotary_reading == LOW && btn_rotary_previous == HIGH && millis() - btn_rotary_time > btn_debounce) {
    menuNav();
    btn_rotary_time = millis();
  }

  btn_rotary_previous = btn_rotary_reading;

}

void menuNav() {

  if (menu_item != 0) {
    traverse_menus = traverse_menus == true ? false : true;
  }

  update = true;

}



/* SIGNAL CAMERA TO TAKE IMAGE(S)
*
* Optionally sends multiple delay seperated signals for exposure bracketing each focus slice
*
*/
void captureImages() {

  for (byte i = 1; i <= settings[5].value; i++) {

    pause(1000); // Allow vibrations to settle

    if (settings[5].value > 1) { // If more than one image is being taken, display the current position in the bracket
      screenPrintProgress();
      screen.setPrintPos(0, 4);
      screen.print(F("Bracket "));
      screen.print(i);
      screen.print(F("/"));
      screen.print(settings[5].value);
    }

    screenPrintProgress();
    shutter(); // Take the image
    screen.print(F("Pause"));

    for (byte i = 1; i <= settings[3].value; i++) {

      pause(1000);
      screenPrintProgress();
      screen.setPrintPos(0, 4);
      screen.print(F("Resume in "));
      screen.print(settings[3].value - i); // Print number of seconds remaining
      screen.print(F("s"));

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

    }

    if (stackCancelled()) break; // Exit early if the stack has been cancelled

  }
}



/* TRIGGER THE CAMERA SHUTTER
*
* Optionally sends two delay seperated signals to support mirror lockup
*
*/
void shutter() {

  screenUpdate();
  screen.setPrintPos(0, 4);
  screen.print(F("  Taking image  "));
  pause(1000);

  for (byte i = 0; i <= settings[4].value; i++) {

    if (settings[4].value) {
      screenPrintProgress();
      screen.setPrintPos(0, 4);
      screen.print(i == 0 ? F("Mirror up") : F("Shutter"));
      pause(500);
    }

    digitalWrite(cam_focus, HIGH); // Trigger camera autofocus - camera may not take picture in some modes if this is not triggered first
    digitalWrite(cam_shutter, HIGH); // Trigger camera shutter

    pause(200); //Small delay needed for camera to process above signals

    digitalWrite(cam_shutter, LOW); // Switch off camera trigger signal
    digitalWrite(cam_focus, LOW); // Switch off camera focus signal

    if (settings[4].value && i == 0) {
      pause(2000); // Pause between mirror up and shutter actuation
    }

    if (stackCancelled()) break; // Exit early if the stack has been cancelled

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
void sysTasks() {

  static byte seconds = 0; // Number of seconds elapsed
  static unsigned long one_second = millis() + 1000; // 1 second from now
  static byte minutes = 0; // Number of minutes elapsed
  static boolean conn_stat_polled = true;

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
    //saveSettings();
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
    //0,-1,1,0,1,0,0,-1,-1,0,0,1,0,1,-1,0
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0 // Use this line instead to increment encoder in the opposite direction
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

  mcp_int_fired = true;
  idle = false;

}



/* PROCESS FLAGGED INTERRUPTS FROM THE MCP23017 */
void handleMcpInterrupt() {

  detachInterrupt(0); // Detach the interrupt while the current instance is dealt with

  byte pin = mcp.getLastInterruptPin(); // Find out which pin on the mcp created the interrupt
  byte val = mcp.getLastInterruptPinValue(); // And get its current state

  if (pin == switch_off_flag) { // If a switchoff signal was recieved from the pushbutton controller
    sysOff();
  }

  if ((pin == step_bwd || pin == step_fwd) && start_stack == false) { // If a manual stage control button was pressed
    stepperDriverManualControl(pin);
  }

  if (pin == btn_rotary) { // Check the current state of the rotary encoder button
    btnRotary();
  }

  EIFR = 0x01; // Clear interrupts
  mcp_int_fired = false; // Reset the flag
  attachInterrupt(0, mcpInterrupt, FALLING); // Re-attach the interrupt

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
void screenPrintProgress() {

  screenUpdate();
  screen.print(F("Slice "));
  screen.print (slice_count);
  screen.print (F("/"));
  screen.print (settings[1].value);

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

      case 'e': // Move stage backward
        stepperDriverManualControl(0);
        break;

      case 'f': // Move stage forward
        stepperDriverManualControl(1);
        break;

      case 'g': // Take a test image
        shutter();
        break;

      case 'h': // Switch Bluetooth on or off
        settings[10].value = settings[10].value == 1 ? 0 : 1;
        update = true;
        break;

      case 'i': //

        break;

      case 'j': // Switch off controller
        sysOff();
        break;

      case 'k': // Keep connection alive
        app_connected = true;
        break;

      default: // Unmatched character - send error then pretend the function call never happened
        Serial.println("");
        Serial.print(F("Command code <"));
        Serial.print(serial_in);
        Serial.print(F("> not recognised"));
        idle = true;
    }
  }
}



/* CHECK IF THE FOCUS STACK HAS BEEN CANCELLED */
boolean stackCancelled() {

  if (!start_stack) {
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
void stepperDriverEnable(boolean enable = true, byte direction = 1, boolean toggle_direction = false) {

  if (enable) {
    mcp.digitalWrite(step_enable, LOW);
    if (toggle_direction) {
      direction = previous_direction;
    }
    digitalWrite(step_dir, direction);
    stepperDriverMicroStepping();
    previous_direction = direction;
  } else if (step_disabled) {
    mcp.digitalWrite(step_enable, HIGH);
  }

}



/* CLEAN UP AFTER FOCUS STACK COMPLETION
*
* Optionally return the stage to its starting position
* Reset various variables to inital values
*
*/
void stackEnd() {

  screenUpdate();
  screen.print(F("Stack "));
  start_stack == false ? screen.print(F("cancelled")) : screen.print(F("finished"));
  delay(2000);

  if (settings[5].value) {

    stepperDriverEnable(true, 0);

    byte slice_size = settings[0].value;
    byte hardware = active_hardware_calibration_setting;
    byte unit_of_measure = uom_multipliers[settings[6].value];
    byte micro_steps = micro_stepping[0][settings[8].value][0];

    //TODO Check this figure is correct
    unsigned int rounded_return_steps = (slice_size * slice_count * hardware * unit_of_measure * micro_steps) - 0.5;

    screenUpdate();
    screen.print(F("Returning"));
    delay(2000);

    stepperDriverEnable(true, 1);

    for (int i; i < rounded_return_steps; i++) {

      stepperDriverStep();
      if (!stepperDriverInBounds()) break;

    }

    if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
    stepperDriverEnable(false);

  }

  menu_item = 0; // Set menu to first option screen
  home_screen = true; // Reinstate the home screen
  slice_count = 0; // Reset pic counter
  start_stack = false; // Return to menu options section
  update = true; // Write the first menu item to the screen

}



/* PULL STAGE AWAY FROM TRIPPED LIMIT SWITCH */
void stepperDriverClearLimitSwitch() {

  screenUpdate();
  screen.print(F("Limit hit"));
  screen.setPrintPos(0, 4);
  screen.print(F("Returning"));
  stepperDriverEnable(true, 0, true); //reverse stepper motor direction

  while (mcp.digitalRead(limit_switch_front) == LOW || mcp.digitalRead(limit_switch_back) == LOW) { //turn stepper motor for as long as  the limit switch remains pressed

    stepperDriverStep();

  }

  stepperDriverEnable(true, 0, true); //restore normal stepper motor direction

  update = true;
  stepperDriverEnable(false);

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



/* MOVE STAGE BACKWARD AND FORWARD */
void stepperDriverManualControl(int direction) {

  char* manual_ctl_strings[2][2] = {{"Back", "<"}, {"For", ">"}};

  if (mcp.digitalRead(direction) == LOW && stepperDriverInBounds()) {
    screenUpdate();
    screen.print(F("Moving "));
    screen.print(manual_ctl_strings[direction][0]);
    screen.print(F("ward"));
    screen.setPrintPos(0, 4);
    for (byte i = 0; i < 16; i++) {
      screen.print(manual_ctl_strings[direction][1]);
    }

    stepperDriverEnable(true, direction);

    while (mcp.digitalRead(direction) == LOW && stepperDriverInBounds()) {
      stepperDriverStep();
    }

  }

  stepperDriverEnable(false);

  if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch();
  update = true; // Go back to displaying the active menu item once manual control button is no longer pressed

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
  delayMicroseconds(settings[7].value * 1000); // Delay time between steps, too short and motor may stall or miss steps

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
void settingUpdate(int &var, int lower, int upper, int multiplier = 1) {

  var = constrain(var, lower, upper); // Keep variable value within specified range

  if (increments != 0) { // If the variable's value was changed at least once since the last check
    var += (multiplier * increments); // Add or subtract from variable
    increments = 0; // Reset for next check
    update = true; // Update menus
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

  if (update) { // Refresh menu content if the active variable has changed

    int menu_var = settings[menu_item].value;
    char print_buffer[16 + 1];
    byte string_length;
    screenUpdate();
    screenPrintMenuArrows(); // Print menu arrows

    switch (menu_item) { // The menu options

      case 0:
        string_length = sprintf(print_buffer, "%s", "Setup");
        break;

      case 1: // Change the number of increments to move each time
        string_length = sprintf(print_buffer, "%d", menu_var);
        break;

      case 2: // Change the number of slices to create in the stack
        string_length = sprintf(print_buffer, "%d", menu_var);
        break;

      case 3: // Change the number of seconds to wait for the camera to capture an image before continuing
        string_length = sprintf(print_buffer, "%ds", menu_var);
        break;

      case 4: // Toggle mirror lockup for the camera
        string_length = menu_var == 1 ? sprintf(print_buffer, repeated_strings[0]) : sprintf(print_buffer, repeated_strings[1]);
        break;

      case 5: // Change the number of images to take per focus slice (exposure bracketing)
        string_length = menu_var == 1 ? sprintf(print_buffer, "%s", repeated_strings[1]) : sprintf(print_buffer, "%d", menu_var);
        break;

      case 6: // Toggle whether camera/subject is returned the starting position at the end of the stack
        string_length = menu_var == 1 ? sprintf(print_buffer, repeated_strings[0]) : sprintf(print_buffer, repeated_strings[1]);
        break;

      case 7: // Select the unit of measure to use for focus slices: Microns, Millimimeters or Centimeters
        string_length = sprintf(print_buffer, "%c%c", uom_chars[menu_var], uom_chars[1]);
        break;

      case 8: // Adjust the stepper motor speed (delay in microseconds between slice_size)
        // A smaller number gives faster motor speed but reduces torque
        // Setting this too low may cause the motor to miss steps or stall
        string_length = sprintf(print_buffer, "%d000uS", menu_var);
        break;

      case 9: // Adjust the degree of microstepping made by the a4988 stepper driver
        // More microsteps give the best stepping resolution but may require more power for consistency and accuracy
        string_length = sprintf(print_buffer, "1/%d", micro_stepping[0][menu_var][0]);
        break;

      case 10: // Toggle power to an external 3.3v bluetooth board e.g. HC-05
        menu_var = !menu_var;
        string_length = sprintf(print_buffer, repeated_strings[menu_var]);
        break;

    }

    stringConstants thisSettingTitle; //Retrieve the setting title from progmem
    memcpy_P(&thisSettingTitle, &settings_titles[menu_item], sizeof thisSettingTitle);

    //Loop through retrieved title and count characters
    //TODO - there has to be a more efficient way of doing this...
    byte string_len = 0;

    for (byte i = 0; i < sizeof thisSettingTitle; i++) {
      if (thisSettingTitle.title[i] != '\0') {
        string_len = i + 1;
      } else {
        break;
      }
    }

    screenPrintCentre(thisSettingTitle.title, string_len, 2); // Print the menu setting title
    if (!traverse_menus) { // Invert the colour of the current menu item to indicate it is editable
      byte boxWidth = (string_length * 8) + 2;
      byte leftPos = ((128 - boxWidth) / 2);
      screen.setMode('~');
      screen.drawBox(leftPos, 50, boxWidth, 20);
      screenPrintCentre(print_buffer, string_length); // Print the menu setting value
    } else {
      screenPrintCentre(print_buffer, string_length); // Print the menu setting value
    }
    update = false;

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
    byte hardware = active_hardware_calibration_setting;
    byte unit_of_measure = uom_multipliers[settings[7].value];
    byte micro_steps = micro_stepping[0][settings[9].value][0];

    slice_count = 1; // Register the first image(s) of the stack is being taken
    screenPrintProgress(); // Print the current position in the stack
    screen.setPrintPos(0, 4);
    screen.print(F("Advance "));
    screen.print(slice_size);
    screen.print(uom_chars[settings[7].value]);
    screen.print(uom_chars[1]);
    captureImages(); // Take the image(s) for the first slice of the stack

    //TODO Check this figure is correct
    unsigned int rounded_steps = (slice_size * hardware * unit_of_measure * micro_steps) - 0.5;

    for (unsigned int i = 1; i < slices; i++) { // For each subsequent focus slice in the stack

      slice_count++; // Record slices made in the stack so far

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      screenPrintProgress(); // Print the current position in the stack
      screen.setPrintPos(0, 4);
      screen.print(F("Advance "));
      screen.print(slice_size);
      screen.print(uom_chars[settings[7].value]);
      screen.print(uom_chars[1]);

      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(true, 1); // Enable the A4988 stepper driver

      for (unsigned int i = 0; i < rounded_steps; i++) {

        stepperDriverStep(); // Send a step signal to the stepper driver
        if (stackCancelled() || !stepperDriverInBounds()) break; // Exit early if the stack has been cancelled or a limit switch is hit

      }

      if (!stepperDriverInBounds()) stepperDriverClearLimitSwitch(); // Clear hit limit switch
      if (stackCancelled()) break; // Exit early if the stack has been cancelled

      stepperDriverEnable(false); // Disable the stepper driver when not in use to save power
      captureImages(); // Take image(s)

    }

    stackEnd(); // End of focus stack tasks

  }
}
