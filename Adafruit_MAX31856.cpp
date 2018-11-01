/*************************************************** 
  This is a library for the Adafruit Thermocouple Sensor w/MAX31856

  Designed specifically to work with the Adafruit Thermocouple Sensor
  ----> https://www.adafruit.com/product/3263
  
  These sensors use SPI to communicate, 4 pins are required to  
  interface
  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/

#include "Adafruit_MAX31856.h"
#ifdef __AVR
  #include <avr/pgmspace.h>
#elif defined(ESP8266)
  #include <pgmspace.h>
#endif

#include <stdlib.h>
#include <SPI.h>

static SPISettings max31856_spisettings = SPISettings(500000, MSBFIRST, SPI_MODE1);


// Software (bitbang) SPI
Adafruit_MAX31856::Adafruit_MAX31856(int8_t spi_mosi, int8_t spi_miso, int8_t spi_clk) {
  _sclk = spi_clk;
  _miso = spi_miso;
  _mosi = spi_mosi;

}

// Hardware SPI init
Adafruit_MAX31856::Adafruit_MAX31856(void) {
  _sclk = _miso = _mosi = -1;
}



boolean Adafruit_MAX31856::begin(int8_t spi_cs) {
  pinMode(spi_cs, OUTPUT);
  digitalWrite(spi_cs, HIGH); // select pin high actually means de-select in SPI

  if (_sclk != -1) {
    //define pin modes
    pinMode(_sclk, OUTPUT); 
    pinMode(_mosi, OUTPUT); 
    pinMode(_miso, INPUT);
  } else {
    //start and configure hardware SPI
    SPI.begin();
  }

  // assert on any fault
  // unmask all fault codes
  // sets _cs to low, writes data and then _cs to high
  writeRegister8(spi_cs,MAX31856_MASK_REG, 0x0); //0x02
  
  // CRO: Configuration bitmask
  // REGISTER: 0x00
  // write [0001 0000] for open circuito fault detection
  writeRegister8(spi_cs,MAX31856_CR0_REG, MAX31856_CR0_OCFAULT0); //0x00,0x10
  setThermocoupleType(spi_cs,MAX31856_TCTYPE_K);

  return true;
}


//void Adafruit_MAX31856::setThermocoupleType(max31856_thermocoupletype_t type) {
void Adafruit_MAX31856::setThermocoupleType(int8_t spi_cs, uint8_t type) {
  uint8_t t = readRegister8(spi_cs,MAX31856_CR1_REG); //0x01
  t &= 0xF0; // mask off bottom 4 bits
  t |= (uint8_t)type & 0x0F;
  writeRegister8(spi_cs,MAX31856_CR1_REG, t);
}


max31856_thermocoupletype_t Adafruit_MAX31856::getThermocoupleType(int8_t spi_cs) {
  uint8_t t = readRegister8(spi_cs, MAX31856_CR1_REG);
  t &= 0x0F;

  return (max31856_thermocoupletype_t)(t);
}


uint8_t Adafruit_MAX31856::readFault(int8_t spi_cs) {
  return readRegister8(spi_cs, MAX31856_SR_REG);
}



void Adafruit_MAX31856::setTempFaultThreshholds(int8_t spi_cs, float flow, float fhigh) {
  int16_t low, high;

  flow *= 16;
  low = flow;

  fhigh *= 16;
  high = fhigh;

  writeRegister8(spi_cs, MAX31856_LTHFTH_REG, high >> 8);
  writeRegister8(spi_cs, MAX31856_LTHFTL_REG, high);

  writeRegister8(spi_cs, MAX31856_LTLFTH_REG, low >> 8);
  writeRegister8(spi_cs, MAX31856_LTLFTL_REG, low);
}


void Adafruit_MAX31856::oneShotTemperature(int8_t spi_cs, int8_t fastFlag) {
  // CJTO: Cold Junction Temperature Offset
  // REGISTER: 0x09
  // if there is a difference in temperature between the cold junction
  // and the cold junction temperature sensor it would make sense to enter
  // a non-zero value here, which will be applied to the cold junction
  // temperature values in register 0x0A and 0x0B.
  // Typically zero makes sense since the sensor and cold junction are in
  // the same physical location.
  writeRegister8(spi_cs, MAX31856_CJTO_REG, 0x0); //0x09

  // CRO: Configuration bitmask
  // REGISTER: 0x00
  uint8_t t = readRegister8(spi_cs, MAX31856_CR0_REG); //0x00

  // &= ![1000 0000]
  // I.e. set Conversion mode to off (default)
  t &= ~MAX31856_CR0_AUTOCONVERT; // turn off autoconvert!
  // |=  [0100 0000]
  // I.e. set it up for a 1-shot conversion
  // This causes a single cold-junction and thermocouple conversion to take place when Conversion
  // Mode bit =0 (normally off mode). The conversion is triggered when CS goes high after writing a 1 to
  // this bit. Note that if a multi-byte write is performed, the conversion is triggered when CS goes high
  // at the end of the transaction. A single conversion requires approximately 143ms in 60Hz filter mode
  // or 169ms in 50Hz filter mode to complete. This bit self clears to 0.
  t |= MAX31856_CR0_1SHOT;

  // write the new configuration
  writeRegister8(spi_cs, MAX31856_CR0_REG, t);  //0x00

  if ( fastFlag != 1 ) {
    delay(250); // MEME FIX autocalculate based on oversampling
  }
}


float Adafruit_MAX31856::readCJTemperature(int8_t spi_cs) {
  oneShotTemperature(spi_cs,0);

  int16_t temp16 = readRegister16(spi_cs, MAX31856_CJTH_REG); //0x0A
  float tempfloat = temp16;
  tempfloat /= 256.0;

  return tempfloat;
}

// readThermocoupleTemperature triggers a 1-shot temperature measurement
// (oneShotTemperature) which includes a 250ms delay.  The MAX31856 datasheet
// indicates that 143 ms would be sufficient when using 60Hz filter, but regardless
// when reading multiple sensors, it doesn't make sense to have this delay for each sensor.
// it would make more sense to trigger each for 1-shot measurement, then read all out after
// a single delay
float Adafruit_MAX31856::readThermocoupleTemperature(int8_t spi_cs) {
  // set it up to take a single temperature measurement
  oneShotTemperature(spi_cs,0);

  // LTCBH:  Linearized TC Temperature, Byte High
  // REGISTER: 0x0C
  // this actually ends up reading 3 bytes, so starts with 0x0C
  // and continues reading the next two LTCBM (0x0D) and LTCBL (0x0E)
  // which are the middle and low bytes
  int32_t temp24 = readRegister24(spi_cs, MAX31856_LTCBH_REG); //0x0C
  
  // I think there is something wrong with this negative correction thing
  // In principle padding with a 1's and bit shift could be made to work, but 
  // I think the code is more readable using a divide by 32 to remove the 5 lowest
  // unused bytes as opposed to sign correction and then bit shifting, and hopeing
  // the bit shifting keeps the correct sign.
#ifdef NEVER
  if (temp24 & 0x800000) {
    temp24 |= 0xFF000000;  // fix sign
  }

  temp24 >>= 5;  // bottom 5 bits are unused

  float tempfloat = temp24;
  tempfloat *= 0.0078125;
#endif
  temp24 /= 32;  // bottom 5 bits are unused
  float tempfloat = temp24;
  tempfloat *= 0.0078125;
  
  return tempfloat;
}


//float Adafruit_MAX31856::readThermocoupleTemperatureFast(int8_t spi_cs) {
void Adafruit_MAX31856::readThermocoupleTemperatureFast(int8_t spi_cs, float *tcTemperature, float *cjTemperature, int8_t fastFlag) {
  // set it up to take a single temperature measurement
  // removed, assume oneShotTemperature initiation by calling code
  if (fastFlag != 1) {
    oneShotTemperature(spi_cs,0);
  }
  uint8_t buffer[5] = {0, 0, 0, 0, 0};
  
  //1st 2 bytes are the CJT, last3 bytes are linearized temperature
  readRegisterN(spi_cs, MAX31856_CJTH_REG, buffer, 5); //0x0A
  
  // 1st 2 bytes are CJT
  uint16_t temp16 = buffer[0];
  temp16 <<= 8;
  temp16 |= buffer[2];
  
  // next 3 bytes are linearized temperature
  uint32_t temp24 = buffer[2];
  temp24 <<= 8;
  temp24 |=  buffer[3];
  temp24 <<= 8;
  temp24 |=  buffer[4];
  temp24 /= 32;  // bottom 5 bits are unused
  
#ifdef NEVER
  float tempfloat = temp24;
  tempfloat *= 0.0078125;
#endif
  
  *tcTemperature = (float)temp24 * 0.0078125;
  *cjTemperature = (float)temp16 / 256;
  
//  return tempfloat;
  
}


/**********************************************/

uint8_t Adafruit_MAX31856::readRegister8(int8_t spi_cs, uint8_t addr) {
  uint8_t ret = 0;
  readRegisterN(spi_cs, addr, &ret, 1);

  return ret;
}


uint16_t Adafruit_MAX31856::readRegister16(int8_t spi_cs, uint8_t addr) {
  uint8_t buffer[2] = {0, 0};
  readRegisterN(spi_cs, addr, buffer, 2);

  uint16_t ret = buffer[0];
  ret <<= 8;
  ret |=  buffer[1];
  
  return ret;
}


uint32_t Adafruit_MAX31856::readRegister24(int8_t spi_cs, uint8_t addr) {
  uint8_t buffer[3] = {0, 0, 0};
  readRegisterN(spi_cs, addr, buffer, 3);

  uint32_t ret = buffer[0];
  ret <<= 8;
  ret |=  buffer[1];
  ret <<= 8;
  ret |=  buffer[2];
  
  return ret;
}


void Adafruit_MAX31856::readRegisterN(int8_t spi_cs, uint8_t addr, uint8_t buffer[], uint8_t n) {
  addr &= 0x7F; // make sure top bit is not set

  if (_sclk == -1)
    SPI.beginTransaction(max31856_spisettings);
  else 
    digitalWrite(_sclk, HIGH);

  digitalWrite(spi_cs, LOW);

  spixfer(addr);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(": ");
  while (n--) {
    buffer[0] = spixfer(0xFF);
    //Serial.print(" 0x"); Serial.print(buffer[0], HEX);
    buffer++;
  }
  //Serial.println();

  if (_sclk == -1)
    SPI.endTransaction();

  digitalWrite(spi_cs, HIGH);
}


void Adafruit_MAX31856::writeRegister8(int8_t spi_cs, uint8_t addr, uint8_t data) {
  addr |= 0x80; // make sure top bit is set

  if (_sclk == -1)
    SPI.beginTransaction(max31856_spisettings);
  else 
    digitalWrite(_sclk, HIGH);

  digitalWrite(spi_cs, LOW);

  spixfer(addr);
  spixfer(data);

  //Serial.print("$"); Serial.print(addr, HEX); Serial.print(" = 0x"); Serial.println(data, HEX);

  if (_sclk == -1)
    SPI.endTransaction();

  digitalWrite(spi_cs, HIGH);
}



uint8_t Adafruit_MAX31856::spixfer(uint8_t x) {
  if (_sclk == -1)
    return SPI.transfer(x);

  // software spi
  //Serial.println("Software SPI");
  uint8_t reply = 0;
  for (int i=7; i>=0; i--) {
    reply <<= 1;
    digitalWrite(_sclk, LOW);
    digitalWrite(_mosi, x & (1<<i));
    digitalWrite(_sclk, HIGH);
    if (digitalRead(_miso))
      reply |= 1;
  }
  return reply;
}
