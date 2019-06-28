/**
 * @file    VCNL4035_Lib.h
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

#ifndef VCNL4035_Lib_H
#define VCNL4035_Lib_H

#include <Arduino.h>
#include "vcnl4035_Defs.h"

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
	uint16_t readVersion();
	bool readPsData(uint16_t *val);
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

	/* Bit manipulation */
	bool writeRegisterBits(uint8_t reg, uint8_t msb_lsb, uint8_t start_bit,
						   uint8_t end_bit, uint8_t val);

	/* Raw I2C reads and writes */
	bool wireWriteByte(uint8_t val);
	bool wireWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte);
	bool wireReadReg(uint8_t reg, uint16_t &val);

	/* Proximity Mode Settings */
	ps_settings_t gesture_mode = {{0}, {0}, {0}, {0}, 0};
	ps_settings_t ps_mode = {{0}, {0}, {0}, {0}, 0};

private:
	uint8_t addr_ = VISHAY_I2C_ADDR;
	//	uint8_t ps_conf_1 = 0, ps_conf_2 = 0, ps_conf_3 = 0, ps_ms = 0;
};

#endif /* VCNL4035_Lib_H */