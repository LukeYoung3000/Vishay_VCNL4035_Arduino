/**
 * @file    VCNL4035_Lib.h
 * @brief   Library for the VCNL3045
 * @author  Luke Young (L S Young Electrical)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the VCNL4035 ambient light and proximity sensor to
 * Arduino over I2C.
 * The library relies on the Arduino Wire (I2C) library. To use the library,
 * instantiate a VCNL4035 object, call init(), and call the desired
 * functions.
 */
 
#include <Arduino.h>
#include <Wire.h>


#include "VCNL4035_Lib.h"


uint16_t VCNL4035::readVersion()
{
	uint16_t ver = 0;
	wireReadReg(VISHAY_VERSION_REG, ver);
	//ver = wireReadReg_2(VISHAY_VERSION_REG);
	
	#if DEBUG
	Serial.print("VCNL4035 Version: ");
	Serial.println(ver,HEX);
	#endif
	
	return ver;
}


/*******************************************************************************
 * Raw I2C Reads and Writes
 * Taken from the ZX_Sensor library from Spark-fun and Modified
 ******************************************************************************/

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool VCNL4035::wireWriteByte(uint8_t val)
{
    Wire.beginTransmission(addr_);
    Wire.write(val);
    if( Wire.endTransmission(false) != 0 ) {
        return false;
    }
    
    return true;
}

/**
 * @brief Writes two bytes to the I2C device and specified register
 *
 * @param[in] reg the register in the I2C device to write to
 * @param[in] low_byte is the lower register bit
 * @param[in] high_byte is the higher register bit
 * @return True if successful write operation. False otherwise.
 */
bool VCNL4035::wireWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte)
{
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(low_byte);
    Wire.write(high_byte);
    if( Wire.endTransmission(false) != 0 ) {
        return false;
    }

    return true;
}

/**
 * @brief Reads a two bytes from the I2C device and specified register
 *
 * @param[in] reg the register to read from
 * @param[out] val the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool VCNL4035::wireReadReg(uint8_t reg, uint16_t &val)
{
	uint16_t bytes[2] = {0,0};
	uint8_t i = 0;

	
    /* Indicate which register we want to read from */
    if (!wireWriteByte(reg)) {
        return false;
    }
    
    /* Read from register */
    Wire.requestFrom(addr_, uint8_t(2));
    while (Wire.available()) {
        // Possible over flow errors here
        bytes[i] = Wire.read();
		    i++;
    }
    
	bytes[1] = (bytes[1] << 8) + bytes[0];
	val = bytes[1];
	
    return true;
}

uint16_t VCNL4035::wireReadReg_2(uint8_t reg)
{
  uint16_t reading;
  Wire.beginTransmission(addr_);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(addr_, uint8_t(2));
  while (!Wire.available());
  uint8_t byteLow = Wire.read();
  while (!Wire.available());
  uint16_t byteHigh = Wire.read();
  reading = (byteHigh <<= 8) + byteLow;
  return reading;
}