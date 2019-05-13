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
 
#ifndef VCNL4035_Lib_H
#define VCNL4035_Lib_H

#include <Arduino.h>

/* Debug */
#define DEBUG 1

/* Acceptable VCNL3045 version */
// Add version numbers!!

/* VCNL3045 I2C slave address */
#define VISHAY_I2C_ADDR   		0x60

/* VCNL3045 register addresses */
#define PS_CONF_1_2   			0x03
#define PS_CONF_3_MS  			0x04
#define RES_INT_FLAGS 			0x0D
#define PS1_DATA_REG  			0x08
#define PS2_DATA_REG  			0x09
#define PS3_DATA_REG  			0x0A
#define VISHAY_VERSION_REG 		0x0E

#define H_BYTE					1
#define L_BYTE					0



/* VCNL4035 Sensor Class */
class VCNL4035 {
public:

	/* High level read functions */
	uint16_t readVersion();
	bool readPsData(uint16_t *val);
	
	/* Bit manipulation */
	bool writeRegisterBits(uint8_t reg, uint8_t msb_lsb, uint8_t start_bit, \
							uint8_t end_bit, uint8_t val);
							
	/* Raw I2C reads and writes */
    bool wireWriteByte(uint8_t val);
	bool wireWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte);
	bool wireReadReg(uint8_t reg, uint16_t &val);
	
	
	
	
private:

	 uint8_t addr_ = VISHAY_I2C_ADDR;

};

#endif /* VCNL4035_Lib_H */