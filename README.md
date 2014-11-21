Stackduino-2
============

Stackduino 2 is an open source motion controller, designed primarily for focus stacking, which interfaces with a stepper motor, limit switches and a camera shutter. It aims to build on the features of the first Stackduino (reallysmall.github.io/Stackduino/) and become more efficient, versatile and portable than its predecessor. 
 
Power
 
Automatic switchover between ac adapter and batteries is supported by an LTC4412 and Mosfet at a much lower forward voltage drop than using diode ORing for greater efficiency of battery use.
 
Step-down to 3.3v VCC is handled with a LM2675 buck converter for considerable efficiency improvement over the standard linear regulator used in Stackduino 1.
 
Power on/ off is supported by the LTC2950 pushbutton controller. This also interfaces with the micro-controller to allow the system to switch itself off if left on inadvertently.
 
Core functions
 
MCP23017 port expander allows software control over most stepper driver functions, power monitoring and system switch-off.

FT232RL interfaces microcontroller with onboard usb port for simple reprogramming.

16x2 parallel lcd replaced with 128x64 OLED, which is more feature rich, smaller, uses less pins and less power.
 
I/O
 
Stackduino 1's DB9 port replaced with a DB15 port, supporting signals for:
 
Ground (3)
Stepper motor (4)
Limit switches (2)
Focus and shutter (2)
Digital pin (2 - currently unused, to support future functionality)
Analogue pin (2 - currently unused, to support future functionality)
