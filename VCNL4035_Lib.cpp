/**
 * @file    VCNL4035_Lib.cpp
 * @brief   Library for the VCNL4035
 * @author  Luke Young (L S Young Electrical)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 *
 * This library interfaces the Vishay VCNL4035 ambient light and proximity
 * sensor to Arduino over I2C.
 * The library relies on the Arduino Wire (I2C) library. To use the library,
 * instantiate a VCNL4035 object, call init(), and call the desired
 * functions.
 */

#include <Arduino.h>
#include <Wire.h>

#include "VCNL4035_Lib.h"

void VCNL4035::init()
{
	/* Initialize I2C */
	Wire.begin();

	/* PS_CONF_1:
	Set Integration Time to 4T (Bit 3:2:1 = 1:1:0)
	Turn on PS (Bit 0 = 0) */
	ps_conf_1 = 0x0c;

	/* PS_CONF_2:
	Turn on GESTURE_INT_EN (Bit 7 = 1)
	Turn on GESTURE_MODE (Bit 6 = 1)
	Set PS_Gain to two step mode (Bit 5:4 = 0:0)
	Set PS_HD to 16bit (Bit 3 = 1) */
	ps_conf_2 = 0xc4;

	/* PS_CONF_3:
	Turn on PS_AF (active force mode) (Bit 3 = 1) */
	ps_conf_3 = 0x08;

	/* PS_MS:
	Set LED Current (LED_I) to 200mA (Bit 0:1:2 = 1:1:1) */
	ps_ms = 0x07;

	wireWriteReg(PS_CONF_1_2, ps_conf_1, ps_conf_2);
	wireWriteReg(PS_CONF_3_MS, ps_conf_3, ps_ms);
}

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

#if VCNL4035_DEBUG
	Serial.print("VCNL4035 Version: ");
	Serial.println(ver, HEX);
#endif

	return ver;
}

/**
 * @brief Reads all 3 PS data registers.
 * Note: This function only works correctly if VCNL is in "Gesture Mode".
 *
 * @param[out] val: pointer to the 0th element of an array of 3 unit16_t's
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
	Wire.requestFrom(addr_, uint8_t(6));
	while (Wire.available())
	{
		// Possible over flow errors here
		bytes[i] = Wire.read();
		i++;
	}

	uint16_t var = 0;
	for (i = 0; i < 6; i++)
	{
		var = (bytes[2 * i + 1] << 8);
		val[i] = var + bytes[2 * i];
	}

#if VCNL4035_DEBUG
	Serial.println("VCNL4035::readPsData");
	Serial.print(val[0]);
	Serial.print(", ");
	Serial.print(val[1]);
	Serial.print(", ");
	Serial.println(val[2]);
#endif

	return true;
}

/**
 * @brief Reads interrupt flag regester on VCNL.
 *
 * @return Value of the interrupt flag regester.
 */
uint8_t VCNL4035::readInterruptFlags()
{
	uint16_t val_16 = 0;
	uint8_t val_8 = 0;
	wireReadReg(RES_INT_FLAGS, val_16);
	val_16 = val_16 >> 8;
	val_8 = val_16;
	return (val_8);
}

/*******************************************************************************
 * High Level Write Functions
 ******************************************************************************/

/**
 * @brief Sets the proximity sensor intergration time.
 *
 * @param[in] Value of new intergration time, See the VCNL4035_PS_IT enum
 * for valid inputs.
 */
void VCNL4035::setPsIntegrationTime(VCNL4035_PS_IT int_time)
{
	writeRegisterBits(PS_CONF_1_2, L_BYTE, PS_IT_START, PS_IT_END, int_time);
}

/**
 * @brief Sets the LED current sink values on VCNL Pin 4,5,6.
 *
 * @param[in] Value of new LED current, See the VCNL4035_LED_I enum
 * for valid inputs.
 */
void VCNL4035::setLedCurrent(VCNL4035_LED_I led_current)
{
	writeRegisterBits(PS_CONF_3_MS, H_BYTE,
					  LED_I_START, LED_I_END, led_current);
}

/**
 * @brief Sets the proximity sensor trigger bit to start gathering data.
 */
void VCNL4035::setPsTrigger()
{
	writeRegisterBits(PS_CONF_3_MS, L_BYTE, 2, 2, 1);
	//ps_conf_3 = 0x0c;
	//wireWriteReg(PS_CONF_3_MS, ps_conf_3, ps_ms);
}

/*******************************************************************************
 * Raw I2C Reads and Writes
 * Taken and Modified from the ZX_Sensor library from Spark-fun
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
	if (Wire.endTransmission(false) != 0)
	{
		return false;
	}

	return true;
}

/**
 * @brief Writes two bytes to the I2C device at specified register
 *
 * @param[in] reg: the register in the I2C device to write to
 * @param[in] low_byte: is the lower register bit
 * @param[in] high_byte: is the higher register bit
 * @return True if successful write operation. False otherwise.
 */
bool VCNL4035::wireWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte)
{
	Wire.beginTransmission(addr_);
	Wire.write(reg);
	Wire.write(low_byte);
	Wire.write(high_byte);
	if (Wire.endTransmission() != 0)
	{
		return false;
	}

	return true;
}

/**
 * @brief Reads two byte value from the I2C device at specified register
 *
 * @param[in] reg: the register to read from
 * @param[out] val: the value returned from the register
 * @return True if successful read operation. False otherwise.
 */
bool VCNL4035::wireReadReg(uint8_t reg, uint16_t &val)
{
	uint16_t bytes[2] = {0, 0};
	uint8_t i = 0;

	/* Indicate which register we want to read from */
	if (!wireWriteByte(reg))
	{
		return false;
	}

	/* Read from register */
	Wire.requestFrom(addr_, uint8_t(2));
	while (Wire.available())
	{
		// Possible over flow errors here
		bytes[i] = Wire.read();
		i++;
	}

	bytes[1] = (bytes[1] << 8) + bytes[0];
	val = bytes[1];

	return true;
}

/**
 * @brief Writes a value to a range of bits (between start_bit and end_bit)
 * whithen a VCNL regester.
 * Note: To set one bit only let, start_bit = end_bit.
 *
 * @param[in] reg: the register to read and write back to
 * @param[in] msb_lsb: Set the byte that your bit range is in (H_BYTE or L_BYTE)
 * @param[in] start_bit: first bit of the range
 * @param[in] end_bit: last bit of the range
 * @param[in] val: value to write to the range of bits
 * @return True if successful write operation. False otherwise.
 */
bool VCNL4035::writeRegisterBits(uint8_t reg, uint8_t msb_lsb,
								 uint8_t start_bit, uint8_t end_bit, uint8_t val)
{
	/* Check for valid bit range */
	int8_t num_bits = end_bit - start_bit;
	if (num_bits < 0)
		return false;

	/* Read current value from register */
	uint16_t current_val = 0;
	if (!wireReadReg(reg, current_val))
	{
		return false;
	}

	/* Create bit mask */
	uint16_t mask = 0;
	uint8_t i;
	for (i = start_bit; i <= end_bit; i++)
	{
		mask = mask + (1 << i);
	}

	/* Shift val to required bit range */
	uint16_t value = 0;
	value = mask & (val << start_bit);

	/* Shift value and mask to required byte range */
	if (msb_lsb)
	{
		mask = mask << 8;
		value = value << 8;
	}

	/* Create new regester value */
	uint16_t new_value = 0;
	new_value = current_val & (~mask);
	new_value = new_value + value;

	/* Seperate into high and low bytes */
	uint16_t low = new_value & 0xff;
	uint16_t high = new_value >> 8;

	/* Write back to the register */
	if (!wireWriteReg(reg, low, high))
	{
		return false;
	}

#if VCNL4035_DEBUG
	Serial.println("Enter: VCNL4035::writeRegisterBits");
	Serial.print("register:    ");
	Serial.println(reg, HEX);
	Serial.print("input_val:   ");
	Serial.println(val, BIN);
	Serial.print("current_val: ");
	Serial.println(current_val, BIN);
	Serial.print("mask:        ");
	Serial.println(mask, BIN);
	Serial.print("new_value:   ");
	Serial.println(new_value, BIN);
	Serial.print("low:         ");
	Serial.println(low, BIN);
	Serial.print("high:        ");
	Serial.println(high, BIN);
	Serial.println("Exit: VCNL4035::writeRegisterBits");
#endif

	return true;
}