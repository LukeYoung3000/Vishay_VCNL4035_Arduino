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

/*******************************************************************************
 * High Level Read Functions
 ******************************************************************************/

/**
 * @brief Reads version number on VCNL4035
 *
 * @return Version number
 */
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

/**
 * @brief Reads all 3 PS data registers
 *
 * @param[out] val: pointer to an array of 3 uint16_t's
 *					val[0] will contain value at PS1_DATA_REG
 *					val[1] will contain value at PS2_DATA_REG
 *					val[2] will contain value at PS3_DATA_REG
 * @return True if successful read operation. False otherwise.
 */
bool VCNL4035::readPsData(uint16_t *val)
{
	uint8_t bytes[10] = {0};
	uint8_t i = 0;

	Wire.beginTransmission(addr_);
    
    /* Read from register */
    Wire.requestFrom(addr_, uint8_t(6) );
    while (Wire.available()) {
        // Possible over flow errors here
        bytes[i] = Wire.read();
		    i++;
    }
    
	uint16_t var = 0;
	for(i = 0; i < 6; i++)
	{
		var = (bytes[2*i+1] << 8);
		val[i] = var + bytes[2*i];
	}
	
	#if DEBUG
	Serial.print(val[0]); Serial.print(", ");
	Serial.print(val[1]); Serial.print(", ");
	Serial.println(val[2]);
	#endif
	
    return true;
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
    if( Wire.endTransmission() != 0 ) {
        return false;
    }

    return true;
}

/**
 * @brief Reads a two bytes from the I2C device at specified register
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


// Re write description
/**
 * @brief clears a bit in a register over I2C
 *
 * @param[in] bit the number of the bit (0-7) to clear
 * @param[in] msb_lsb 1 for most significant byte, 0 for LSB
 * @return True if successful write operation. False otherwise.
 */
bool VCNL4035::writeRegisterBits(uint8_t reg, uint8_t msb_lsb, \
								uint8_t start_bit, uint8_t end_bit, uint8_t val)
{ 
	/* Check for valid bit range */
	int8_t num_bits = end_bit - start_bit;
	if(num_bits < 1)	return false;
	
	
    uint16_t current_val = 0;
    /* Read current value from register */
    if ( !wireReadReg(reg, current_val) ) {
        return false;
    }
    	
	uint16_t mask = 0;
	uint16_t value = 0;
	uint8_t i;
	
	/* Create bit mask */
	for(i = start_bit; i <= end_bit; i++)
	{
		mask = mask + (1 << i);
	}
	
	value = mask & (val << start_bit);
	
	if(msb_lsb)
	{
		mask = mask << 8;
		value = value << 8;
	}
	
	uint16_t new_value = 0;
	new_value = current_val & (~mask);
	new_value = new_value + value;
	
	uint16_t low = new_value & 0xff;
	uint16_t high = new_value >> 8; 
	
	
    /* Write back to the register */
    if ( !wireWriteReg(reg, low, high) ) {
        return false;
    }
	
	#if DEBUG
	Serial.print("current_val: "); Serial.println(current_val,BIN);
	Serial.print("mask:        "); Serial.println(mask,BIN);
	Serial.print("new_value:   "); Serial.println(new_value,BIN);
	Serial.print("low:         "); Serial.println(low,BIN);
	Serial.print("high:        "); Serial.println(high,BIN);
	#endif
	 
    return true;    
}