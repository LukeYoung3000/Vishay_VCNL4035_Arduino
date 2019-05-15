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

/* Debug */
#define VCNL4035_DEBUG 0


/* Acceptable VCNL4035 version */
#define VISHAY_VERSION_LSB 0x80
#define VISHAY_VERSION_LSB 0x00

/* VCNL4035 I2C slave address */
#define VISHAY_I2C_ADDR 0x60

/* VCNL4035 I2C register addresses */
#define ALS_CONF_1_2 		0x00
#define ALS_THDH_1_2 		0x01
#define ALS_THDL_1_2 		0x02
#define PS_CONF_1_2 		0x03
#define PS_CONF_3_MS 		0x04
#define PS_CANC_L_M 		0x05
#define PS_THDL_L_M 		0x06
#define PS_THDH_L_M 		0x07
#define PS1_DATA_REG 		0x08
#define PS2_DATA_REG 		0x09
#define PS3_DATA_REG 		0x0A
#define ALS_DATA_L_M 		0x0B
#define WHITE_DATA_L_M 		0x0C
#define RES_INT_FLAGS 		0x0D
#define VISHAY_VERSION_REG 	0x0E

#define H_BYTE 1
#define L_BYTE 0

/* VCNL4035 register parameters */
typedef enum VCNL4035_PS_CONF_1
{
	PS_SD = 0,
	PS_IT_START = 1,
	PS_IT_END = 3,
	PS_PRES_START = 4,
	PS_PRES_END = 5,
	PS_DUTY_START = 6,
	PS_DUTY_END = 7,
} VCNL4035_PS_CONF_1;

typedef enum VCNL4035_PS_CONF_2
{
	PS_INT_START = 0,
	PS_INT_END = 1,
	PS_NS = 2,
	PS_HD = 3,
	PS_GAIN_START = 4,
	PS_GAIN_END = 5,
	GESTURE_MODE = 6,
	GESTURE_INT_EN = 7,
} VCNL4035_PS_CONF_2;

typedef enum VCNL4035_PS_CONF_3
{
	PS_SC_EN = 0,
	PS_MS = 1,
	PS_TRIG = 2,
	PS_AF = 3,
	PS_SMART_PERS = 4,
	IRED_SELECT_START = 5,
	IRED_SELECT_END = 6,
	LED_I_LOW = 7,
} VCNL4035_PS_CONF_3;

typedef enum VCNL4035_PS_MS
{
	LED_I_START = 0,
	LED_I_END = 2,
	PS_SPO = 3,
	PS_SP = 4,
	PS_SC_CUR_START = 5,
	PS_SC_CUR_END = 6,
	RESERVED = 7,
} VCNL4035_PS_MS;

/* Proximity Sensor Integration (T = 50 micro sec) */
typedef enum VCNL4035_PS_IT
{
	PS_IT_1T = 0,
	PS_IT_1_5T = 1,
	PS_IT_2T = 2,
	PS_IT_2_5T = 3,
	PS_IT_3T = 4,
	PS_IT_3_5T = 5,
	PS_IT_4T = 6,
	PS_IT_8T = 7,
} VCNL4035_PS_IT;

/* Proximity Sensor LED Current */
typedef enum VCNL4035_LED_I
{
	LED_I_50_mA = 0,
	LED_I_75_mA = 1,
	LED_I_100_mA = 2,
	LED_I_120_mA = 3,
	LED_I_140_mA = 4,
	LED_I_160_mA = 5,
	LED_I_180_mA = 6,
	LED_I_200_mA = 7,
} VCNL4035_LED_I;

/* VCNL4035 Sensor Class */
class VCNL4035
{
public:
	void init();

	/* High level read functions */
	uint16_t readVersion();
	bool readPsData(uint16_t *val);
	uint8_t readInterruptFlags();

	/* High level write functions */
	void setPsIntegrationTime(VCNL4035_PS_IT int_time);
	void setLedCurrent(VCNL4035_LED_I led_current);
	void setPsTrigger();

	/* Bit manipulation */
	bool writeRegisterBits(uint8_t reg, uint8_t msb_lsb, uint8_t start_bit,
						   uint8_t end_bit, uint8_t val);

	/* Raw I2C reads and writes */
	bool wireWriteByte(uint8_t val);
	bool wireWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte);
	bool wireReadReg(uint8_t reg, uint16_t &val);

private:
	uint8_t addr_ = VISHAY_I2C_ADDR;
	uint8_t ps_conf_1 = 0, ps_conf_2 = 0, ps_conf_3 = 0, ps_ms = 0;
};

#endif /* VCNL4035_Lib_H */