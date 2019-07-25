/**
 * @file    VCNL4035_Lib.h
 * @brief   Library for the VCNL4035
 * @author  Luke Young (L S Young Electrical)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 * 
 * The libray provides a class to interface with the Vishay VCNL4035 ambient
 * light and proximity sensor and abstact the I2C communications from the
 * software developer. The accompanying .cpp files are seperated
 * into "VCNL4035_Lib.cpp" and "VCNL4035_Lib_Hardware.cpp".
 * "VCNL4035_Lib.cpp" contains the portable functions the libray uses.
 * "VCNL4035_Lib_Hardware.cpp" contains all the hardware specific I2C functions.
 * Currently this file has been designed to work with the Arduino
 * Wire librays I2C interface however, it can be adapted to suit any micro
 * controllers I2C functions.
 */

#ifndef VCNL4035_Lib_H
#define VCNL4035_Lib_H

#include <Arduino.h>
#include "VCNL4035_Defs.h"

/* VCNL4035 Sensor Class */
class VCNL4035
{
public:
	/* Setup Functions */
	VCNL4035();
	void init(VCNL4035_MODES mode);

	/* High Level Settings Functions */
	void updatePsSettings(ps_settings_t set);
	void setGestureMode();
	void setPSMode();

	/* High level read functions */
	void readGestureData(uint16_t *val);
	// Make the functions bellow in the .cpp file
	// uint16_t readPS1Data();
	// uint16_t readPS2Data();
	// uint16_t readPS3Data();
	uint16_t readVersion();
	uint8_t readInterruptFlags();

	/* High level write functions */
	void setPsIntegrationTime(VCNL4035_PS_IT int_time);
	void setLedCurrent(VCNL4035_LED_I led_current);
	void setPsTrigger();
	void clearPsCloseFlag();
	void setPsInterruptThresholdLow(uint16_t val);
	void setPsInterruptThresholdHigh(uint16_t val);
	// Make the functions bellow in the .cpp file
	// void setPsDutyCycle(VCNL4035_PS_DUTY duty_cycle);
	// void setPsShutdown(VCNL4035_PS_SD shutdown);
	// void setPsDataResolution(VCNL4035_PS_HD resolution);
	// void setPsActiveForce(VCNL4035_PS_AF on_off);
	// void setPsIredSelect(VCNL4035_IRED_SELECT ired);

	/* Hardware Specfic Functions (VCNL4035_Lib_Hardware.cpp) */
	/* Raw I2C reads and writes */
	void i2cBegin();
	bool i2cWriteRegBits(uint8_t reg, uint8_t msb_lsb, uint8_t start_bit,
						 uint8_t end_bit, uint8_t val);
	bool i2cWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte);
	bool i2cWriteByte(uint8_t val);
	bool i2cReadReg(uint8_t reg, uint16_t &val);
	bool i2cReadPsData(uint16_t *val);

	/* Hardware Specfic Functions (VCNL4035_Lib_Hardware.cpp) */
	/* Hardware Interrupt Handling */
	bool readInt();
	void setIntPin(uint8_t int_pin);

	/* Proximity/Gesture Mode Settings */
	ps_settings_t gesture_mode = {{0}, {0}, {0}, {0}, 0};
	ps_settings_t ps_mode = {{0}, {0}, {0}, {0}, 0};

private:
	uint8_t addr_ = VISHAY_I2C_ADDR;
	uint8_t int_pin_ = 0;
};

#endif /* VCNL4035_Lib_H */