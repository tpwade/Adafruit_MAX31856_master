This is the Adafruit MAX31856 Arduino Library 

Tested and works great with the Adafruit Thermocouple Breakout w/MAX31856
    
   * https://www.adafruit.com/product/3263

These sensors use SPI to communicate, 4 pins are required to  
interface

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Limor Fried/Ladyada  for Adafruit Industries.  
BSD license, check license.txt for more information
All text above must be included in any redistribution



Modified by Trevor Wade, Oct 2018
Main differences are:
* that the device select pin (_cs) is no longer being hard-coded, and is now 
  provided as an input variable
* now provide a fast read option.  Basically the sensors need some delay 
  (~150us) after a read is triggered.  This option allows to trigger all
  sensors and then have bulk delay, and is generally better for multiple 
  sensors

For information on installing libraries, see: http://www.arduino.cc/en/Guide/Libraries
