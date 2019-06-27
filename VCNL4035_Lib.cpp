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

VCNL4035::VCNL4035(void)
{
	/* Gesture Mode Default Settings */
	gesture_mode.ps_conf_1.ps_it = PS_IT_50us;
	gesture_mode.ps_conf_2.ps_hd = PS_HD_16_BIT;
	gesture_mode.ps_conf_2.gesture_mode = GESTURE_MODE_ENABLE;
	gesture_mode.ps_conf_2.gesture_int_en = GESTURE_INT_EN_ENABLE;
	gesture_mode.ps_conf_3.ps_af = PS_AF_ON;
	gesture_mode.ps_ms.led_i = LED_I_50_mA;
	gesture_mode.ps_thdl = 0;
	gesture_mode.ps_thdh = 0xffff;

	/* Proximity Mode Default Settings */
	ps_mode.ps_conf_1.ps_it = PS_IT_50us;
	ps_mode.ps_conf_1.ps_pers = PS_PERS_4_READING;
	ps_mode.ps_conf_1.ps_duty = PS_DUTY_1_320;
	ps_mode.ps_conf_2.ps_int = PS_INT_CLOSING;
	ps_mode.ps_conf_2.ps_ns = PS_NS_TWO_STEP_MODE_TIMES_1;
	ps_mode.ps_conf_2.ps_hd = PS_HD_16_BIT;
	ps_mode.ps_conf_2.ps_gain = PS_GAIN_TWO_STEP_MODE;
	ps_mode.ps_conf_3.ired_select = IRED_SELECT_1;
	ps_mode.ps_ms.led_i = LED_I_50_mA;
	ps_mode.ps_ms.ps_sc_cur = PS_SC_CUR_TYPICAL_TIMES_1;
	ps_mode.ps_thdl = 15;
	ps_mode.ps_thdh = 50;
}

void VCNL4035::init(VCNL4035_MODES mode)
{
	/* Initialize I2C */
	Wire.begin();

	/* Sets Operational Mode */
	switch (mode)
	{
	case GESTURE:
		setGestureMode();
		break;

	case PROXIMITY_SENSOR:
		setPSMode();
		break;

	default:
		setPSMode();
	}

	/* Gather Backround Noise Data From VCNL */
	// We will need the users Int time and LED current in order to find this.
}

/*******************************************************************************
 * High Level Settings Functions
 ******************************************************************************/

/**
 * @brief Sets VCNL4035 proximity registers to the values specified by "set".
 */
void VCNL4035::updatePsSettings(ps_settings_t set)
{
	// PS_CONF_1:
	uint8_t ps_sd = (set.ps_conf_1.ps_sd << PS_SD);
	uint8_t ps_it = (set.ps_conf_1.ps_it << PS_IT_START);
	uint8_t ps_pers = (set.ps_conf_1.ps_pers << PS_PRES_START);
	uint8_t ps_duty = (set.ps_conf_1.ps_duty << PS_DUTY_START);
	uint8_t ps_conf_1 = ps_sd + ps_it + ps_pers + ps_duty;

	// PS_CONF_2:
	uint8_t ps_int = (set.ps_conf_2.ps_int << PS_INT_START);
	uint8_t ps_ns = (set.ps_conf_2.ps_ns << PS_NS);
	uint8_t ps_hd = (set.ps_conf_2.ps_hd << PS_HD);
	uint8_t ps_gain = (set.ps_conf_2.ps_gain << PS_GAIN_START);
	uint8_t gesture_mode = (set.ps_conf_2.gesture_mode << GESTURE_MODE);
	uint8_t gesture_int_en = (set.ps_conf_2.gesture_int_en << GESTURE_INT_EN);
	uint8_t ps_conf_2 = ps_int + ps_ns + ps_hd + ps_gain + gesture_mode + gesture_int_en;

	// PS_CONF_3:
	uint8_t ps_sc_en = (set.ps_conf_3.ps_sc_en << PS_SC_EN);
	uint8_t ps_ms_command = (set.ps_conf_3.ps_ms << PS_MS);
	uint8_t ps_trig = (set.ps_conf_3.ps_trig << PS_TRIG);
	uint8_t ps_af = (set.ps_conf_3.ps_af << PS_AF);
	uint8_t ps_smart_pers = (set.ps_conf_3.ps_smart_pers << PS_SMART_PERS);
	uint8_t ired_select = (set.ps_conf_3.ired_select << IRED_SELECT_START);
	uint8_t led_i_low = (set.ps_conf_3.led_i_low << LED_I_LOW);
	uint8_t ps_conf_3 = ps_sc_en + ps_ms_command + ps_trig + ps_af + ps_smart_pers + ired_select + led_i_low;

	// PS_MS:
	uint8_t led_i = (set.ps_ms.led_i << LED_I_START);
	uint8_t ps_spo = (set.ps_ms.ps_spo << PS_SPO);
	uint8_t ps_sp = (set.ps_ms.ps_sp << PS_SP);
	uint8_t ps_sc_cur = (set.ps_ms.ps_sc_cur << PS_SC_CUR_START);
	uint8_t ps_ms = led_i + ps_spo + ps_sp + ps_sc_cur;

	// PS_THDL:
	uint8_t ps_thdl_l = (set.ps_thdl & 0xff);
	uint8_t ps_thdl_m = (set.ps_thdl >> 8);

	// PS_THDH:
	uint8_t ps_thdh_l = (set.ps_thdh & 0xff);
	uint8_t ps_thdh_m = (set.ps_thdh >> 8);

	wireWriteReg(PS_THDL_L_M, ps_thdl_l, ps_thdl_m);
	wireWriteReg(PS_THDH_L_M, ps_thdh_l, ps_thdh_m);
	wireWriteReg(PS_CONF_1_2, ps_conf_1, ps_conf_2);
	wireWriteReg(PS_CONF_3_MS, ps_conf_3, ps_ms);
}

/**
 * @brief Sets VCNL4035 registers to gesture mode
 */
void VCNL4035::setGestureMode()
{
	updatePsSettings(gesture_mode);
}

/**
 * @brief Sets VCNL4035 registers to normal proximity mode
 */
void VCNL4035::setPSMode()
{
	updatePsSettings(ps_mode);
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
 * @brief Sets the proximity sensor integration time.
 *
 * @param[in] Value of new integration time, See the VCNL4035_PS_IT enum
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

/**
 * @brief Clears the interrupt flag on the VCNL that is set when an object is 
 * detected closer than the high thershold value.
 */
void VCNL4035::clearPsCloseFlag()
{
	writeRegisterBits(RES_INT_FLAGS, H_BYTE, PS_IF_CLOSE, PS_IF_CLOSE, 0);
}

void VCNL4035::setPsInterruptThresholdLow(uint16_t val)
{
	uint8_t ps_thdl_l = (val & 0xff);
	uint8_t ps_thdl_m = (val >> 8);
	wireWriteReg(PS_THDL_L_M, ps_thdl_l, ps_thdl_m);
}

void VCNL4035::setPsInterruptThresholdHigh(uint16_t val)
{
	uint8_t ps_thdh_l = (val & 0xff);
	uint8_t ps_thdh_m = (val >> 8);
	wireWriteReg(PS_THDH_L_M, ps_thdh_l, ps_thdh_m);
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
 * whiten a VCNL register.
 * Note: To set one bit only, let start_bit = end_bit.
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

	/* Create new register value */
	uint16_t new_value = 0;
	new_value = current_val & (~mask);
	new_value = new_value + value;

	/* Separate into high and low bytes */
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