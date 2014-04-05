//--- THIS TOOL ONLY FOR DIGOLE SERIAL COLOR OLED MODULE!

/* Design welcome screen and Micro Commands for Digole Serial Color OLED
* This module use different way to show welcome screen, other than previouse module which only display bitmap,
* this module can use any internal commands designed for it when display welcome screen. The size of welcome
* screen still only have 1920 bytes, we used this technical to do this, the data structure of welcome screen is:
* first 2 bytes are the rest of data in welcome screen, as integer, 1st is low byte, 2nd is high byte
* the rest of data are welcome screen commands, so the total size of welcome screen is: size in bytes of commands+2
* We have some tools to help you design welcome screen and Micro Commands set, the steps are:
* 1) design characters which you want to display, you can send the color, font size, display position
* 2) if you want to draw circle, rectangle, pie, lines, please add the correspond commands
* 3) if you want to display picture, please use our picture to C data tool to convert your picture data:
*    www.digole.com/tools/PicturetoC_Hex_converter.php
* 4) add corresponds display picture command and add picture data follow by the commands, there are 3 commands for this module:
*    "DIM", "EDIM1", "EDIM3", for more detail of these commands, please refer to program manual
* 5) put all commands and data together, then use www.digole.com/tools/Convert_C_Format_String_to_Array.php tool to convert the
*    data to array, WHY? because Arduino can't accept long string variable, but accept long array
* 6) put the converted array data in wel[] variable, then run this program, this program will display your welcome screen first,
*    then downloading it to the Flash memory in module, SUGGESTION: just download finished welcome screen, so, on testing period,
*    please comment all downloading routing.
*  After you downloaded the welcome screen or micro commands set, you don't need to do it in the program of production, it will save
*  lot of memory space in the production's software.
*
*  SAMPLE: digole's welcome screen commands set:
*   "CLSF\x78 SC\xf1 ETP\x22\x29 TTWelcome\x0d SF\x0a SC\x13 TRT TRT ETP\x08\x3fTTDigole Digital Solutions\x0dSC\x1f SF\x01 TRT
TTYou will find it's so easy to use and fun:\x0dSC\xE0TTwww.Digole.com\rEDIM1\x00\x10\x1e\x1e....(logo data)", which converted to array bellow
*/
#define _Digole_Serial_UART_  //To tell compiler compile the special communication only, 
//all available are:_Digole_Serial_UART_, _Digole_Serial_I2C_ and _Digole_Serial_SPI_
#include <DigoleSerial.h>
//--------UART setup, if you don't use UART, use // to comment following line
#if defined(_Digole_Serial_UART_)
DigoleSerialDisp mydisp(&Serial, 9600); //UART:Arduino UNO: Pin 1(TX)on arduino to RX on module
#endif
//--------I2C setup, if you don't use I2C, use // to comment following 2 lines
#if defined(_Digole_Serial_I2C_)
#include <Wire.h>
DigoleSerialDisp mydisp(&Wire,'\x27');  //I2C:Arduino UNO: SDA (data line) is on analog input pin 4, and SCL (clock line) is on analog input pin 5 on UNO and Duemilanove
#endif
//--------SPI setup, if you don't use SPI, use // to comment following line
#if defined(_Digole_Serial_SPI_)
DigoleSerialDisp mydisp(8,9,10);  //SPI:Pin 8: data, 9:clock, 10: SS, you can assign 255 to SS, and hard ground SS pin on module
#endif

prog_uchar wel[] PROGMEM ={
40,4,
67,76,13,10,69,68,73,77,49,0,16,30,30,0,0,0,0,0,0,0,0,217,180,181,0,0,0,74,2,75,75,147,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,218,
181,217,181,181,0,0,74,38,2,1,1,41,110,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,218,254,254,177,145,0,0,74,2,111,147,73,5,1,1,110,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,218,181,217,181,145,0,0,74,38,0,0,0,0,78,37,1,74,0,0,0,0,0,0,0,0,0,0,183,146,0,0,145,145,104,0,0,0,74,38,0,0,
0,0,0,0,74,1,37,183,0,0,0,0,0,0,0,183,73,0,0,0,0,0,0,0,0,0,74,37,0,0,0,0,0,0,0,110,37,37,0,0,0,0,0,0,0,74,1,110,0,0,
0,0,0,0,0,0,74,74,0,0,0,0,0,0,0,0,110,1,37,0,0,0,0,0,110,1,110,0,0,0,0,0,0,0,0,0,74,37,0,0,0,0,0,0,0,0,0,74,1,110,
0,0,0,0,37,37,0,0,0,0,0,0,0,0,0,0,74,70,0,0,0,0,0,0,0,0,0,0,37,37,0,0,0,110,37,110,0,0,0,0,0,0,0,0,0,0,74,70,0,0,
0,0,0,0,0,0,0,0,110,1,110,0,0,74,1,0,0,0,0,0,0,0,0,0,0,0,74,69,0,0,0,0,0,0,0,0,0,0,0,37,37,0,111,37,74,0,0,0,0,0,
0,0,0,0,0,0,74,69,0,0,0,0,0,0,0,0,0,0,0,73,1,0,42,6,110,0,0,0,0,0,0,0,0,0,0,0,74,37,0,0,0,0,0,0,0,0,0,0,0,146,
1,74,38,2,147,0,0,0,0,0,0,0,0,0,0,0,74,38,0,0,0,0,0,0,0,0,0,0,0,147,37,37,6,6,151,0,0,0,0,0,0,0,0,0,0,0,74,38,0,0,
0,0,0,0,0,0,0,0,0,147,5,6,39,2,147,0,0,0,0,0,0,0,0,0,0,0,74,38,0,0,0,0,0,0,0,0,0,0,0,147,1,38,38,38,115,0,0,0,0,0,
0,0,0,0,0,0,74,38,0,0,0,0,0,0,0,0,0,0,0,147,1,37,75,1,147,0,0,0,0,0,0,0,0,0,0,0,74,38,0,0,0,0,0,0,0,0,0,0,0,147,
37,37,111,1,74,0,0,0,0,0,0,0,0,0,0,0,74,38,0,0,0,0,0,0,0,0,0,0,0,73,1,0,0,37,1,0,0,0,0,0,0,0,0,0,0,0,74,37,0,0,
0,0,0,0,0,0,0,0,0,73,37,0,0,110,37,110,0,0,0,0,0,0,0,0,0,0,74,37,0,0,0,0,0,0,0,0,0,0,110,37,110,0,0,0,1,37,0,0,0,0,
0,0,0,0,0,0,74,37,0,0,0,0,0,0,0,0,0,0,69,0,0,0,0,0,110,1,74,0,0,0,0,0,0,0,0,0,74,37,0,0,0,0,0,0,0,0,0,110,33,110,
0,0,0,0,0,69,0,110,0,0,0,0,0,0,0,0,74,38,0,0,0,0,0,0,0,0,142,33,73,0,0,0,0,0,0,0,37,0,146,0,0,0,0,0,0,0,74,38,0,0,
0,0,0,0,0,146,32,37,0,0,0,0,0,0,0,0,0,33,0,110,0,0,0,0,0,0,74,70,0,0,0,117,81,154,0,0,73,179,0,0,0,0,0,0,0,0,0,0,73,0,
69,74,0,0,0,0,70,38,0,0,149,153,121,44,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,142,37,37,37,73,146,147,74,74,0,0,149,121,121,48,4,0,0,0,0,0,
0,0,0,0,0,0,0,0,0,0,0,110,37,1,33,1,70,37,0,0,117,85,84,12,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,74,38,1,74,0,0,
4,12,12,117,0,0,0,0,0,0,0,0,13,10,83,70,120,32,83,67,241,32,69,84,80,34,41,32,84,84,87,101,108,99,111,109,101,13,32,83,70,10,32,83,67,19,32,84,82,84,
32,84,82,84,32,69,84,80,8,63,84,84,68,105,103,111,108,101,32,68,105,103,105,116,97,108,32,83,111,108,117,116,105,111,110,115,13,83,67,31,32,83,70,1,32,84,82,84,84,84,
89,111,117,32,119,105,108,108,32,102,105,110,100,32,105,116,39,115,32,115,111,32,101,97,115,121,32,116,111,32,117,115,101,32,97,110,100,32,102,117,110,58,13,83,67,224,84,84,119,119,
119,46,68,105,103,111,108,101,46,99,111,109,13};

#define _DOWNLOAD_MICRO_COMMANDS_ 0;  //1:download welcome screen, 2:download micro commands set

void setup() {
  mydisp.begin();
  mydisp.clearScreen(); //CLear screen
    for (int j = 0; j < sizeof(wel);j++) {
         mydisp.write(pgm_read_byte_near(wel+j));  
    }


         mydisp.write(0);
         mydisp.write(0);
  delay(2000);
#if defined(_DOWNLOAD_WELCOME_)
  //--- start downloading welcome screen to module, comment when testing
  mydisp.uploadStartScreen(sizeof(wel),wel);
  //end downloading
#endif
#if defined(_DOWNLOAD_MICRO_COMMANDS_)
  //--- start downloading welcome screen to module, comment when testing
  mydisp.uploadStartScreen(sizeof(wel),wel);
  //end downloading
#endif
}
void loop() {
}


