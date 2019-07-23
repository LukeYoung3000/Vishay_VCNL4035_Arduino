/**
 * @file    vcnl4035_Defs.h
 * @brief   Arduino Library for the VCNL4035 - Definitions 
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

#ifndef VCNL_4035_Defs_H
#define VCNL_4035_Defs_H

#include <Arduino.h>

/* Debug */
#define VCNL4035_DEBUG 0

/* Acceptable VCNL4035 version */
#define VISHAY_VERSION_LSB 0x80
#define VISHAY_VERSION_LSB 0x00

/* VCNL4035 I2C slave address */
#define VISHAY_I2C_ADDR 0x60

/* VCNL4035 I2C register addresses */
#define ALS_CONF_1_2 0x00
#define ALS_THDH_1_2 0x01
#define ALS_THDL_1_2 0x02
#define PS_CONF_1_2 0x03
#define PS_CONF_3_MS 0x04
#define PS_CANC_L_M 0x05
#define PS_THDL_L_M 0x06
#define PS_THDH_L_M 0x07
#define PS1_DATA_REG 0x08
#define PS2_DATA_REG 0x09
#define PS3_DATA_REG 0x0A
#define ALS_DATA_L_M 0x0B
#define WHITE_DATA_L_M 0x0C
#define RES_INT_FLAGS 0x0D
#define VISHAY_VERSION_REG 0x0E

#define H_BYTE 1
#define L_BYTE 0

/* VCNL4035 Operational Modes */
typedef enum VCNL4035_MODES
{
    GESTURE = 0,
    PROXIMITY_SENSOR = 1,
} VCNL4035_MODES;

/* VCNL4035 Register Bit Locations */
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

typedef enum VCNL4035_INT_FLAG
{
    PS_IF_AWAY = 0,
    PS_IF_CLOSE = 1,
    PS1_SPFLAG = 2,
    PS2_SPFLAG = 3,
    ALS_IF_H = 4,
    ALS_IF_L = 5,
    PS3_SPFLAG = 6,
    GESTURE_DATA_READY_FLAG = 7,
} VCNL4035_INT_FLAG;

/* VCNL Command Parrameters */

/* PS CONF 1 */

/* Proximity Sensor Power Shutdown */
typedef enum VCNL4035_PS_SD
{
    PS_SD_POWER_ON = 0,       // Turns on proximity sensor
    PS_SD_POWER_SHUTDOWN = 1, // Shutdown proximity sensor
} VCNL4035_PS_SD;

/* Proximity Sensor Integration (T = 50 micro sec) */
typedef enum VCNL4035_PS_IT
{
    PS_IT_50us = 0,  // 1.0 T
    PS_IT_75us = 1,  // 1.5 T
    PS_IT_100us = 2, // 2.0 T
    PS_IT_125us = 3, // 2.5 T
    PS_IT_150us = 4, // 3.0 T
    PS_IT_175us = 5, // 3.5 T
    PS_IT_200us = 6, // 4.0 T
    PS_IT_400us = 7, // 8.0 T
} VCNL4035_PS_IT;

/* Proximity Sensor Interrupt Persistance */
typedef enum VCNL4035_PS_PERS
{
    PS_PERS_1_READING = 0,
    PS_PERS_2_READING = 1,
    PS_PERS_3_READING = 2,
    PS_PERS_4_READING = 3,
} VCNL4035_PS_PERS;

/* Proximity Sensor Duty Cycle */
typedef enum VCNL4035_PS_DUTY
{
    PS_DUTY_1_40 = 0,  // 40 off pulses for every 1 on pulse
    PS_DUTY_1_80 = 1,  // 80 off pulses for every 1 on pulse
    PS_DUTY_1_160 = 2, // 160 off pulses for every 1 on pulse
    PS_DUTY_1_320 = 3, // 320 off pulses for every 1 on pulse
} VCNL4035_PS_DUTY;

/* PS CONF 2 */

/* Proximity Sensor Interrupt Trigger Mode */
typedef enum VCNL4035_PS_INT
{
    PS_INT_DISABLE = 0,
    PS_INT_CLOSING = 1,
    PS_INT_AWAY = 2,
    PS_INT_CLOSING_AWAY = 3,
} VCNL4035_PS_INT;

/* Proximity Sensor Sensitivity */
typedef enum VCNL4035_PS_NS
{
    PS_NS_TWO_STEP_MODE_TIMES_4 = 0,
    PS_NS_TWO_STEP_MODE_TIMES_1 = 1,
} VCNL_PS_NS;

/* Proximity Sensor Data Resolution */
typedef enum VCNL4035_PS_HD
{
    PS_HD_12_BIT = 0, // 12 bit resolution
    PS_HD_16_BIT = 1, // 16 bit resolution
} VCNL4035_PS_HD;

/* Proximity Sensor Gain Modes */
typedef enum VCNL4035_PS_GAIN
{
    PS_GAIN_TWO_STEP_MODE = 0,
    PS_GAIN_SINGLE_MODE_8 = 2,
    PS_GAIN_SINGLE_MODE_1 = 3,
} VCNL4035_PS_GAIN;

/* Proximity Sensor Gesture Mode */
typedef enum VCNL4035_GESTURE_MODE
{
    GESTURE_MODE_DISABLE = 0,
    GESTURE_MODE_ENABLE = 1,
} VCNL4035_GESTURE_MODE;

/* Proximity Sensor Gesture Mode Interrupt Enable */
typedef enum VCNL4035_GESTURE_INT_EN
{
    GESTURE_INT_EN_DISABLE = 0,
    GESTURE_INT_EN_ENABLE = 1,
} VCNL4035_GESTURE_INT_EN;

/* PS CONF 3 */

/* Proximity Sensor Sunlight Cancel Enable */
typedef enum VCNL4035_PS_SC_EN
{
    PS_SC_EN_OFF = 0,
    PS_SC_EN_ON = 1,
} VCNL4035_PS_SC_EN;

/* Proximity Sensor Interrupt Pin Functionally */
typedef enum VCNL4035_PS_MS_COMMAND
{
    PS_MS_NORMAL_MODE = 0,
    PS_MS_LOGIC_MODE = 1,
} VCNL4035_PS_MS_COMMAND;

/* Proximity Sensor Active Force Mode Trigger */
typedef enum VCNL4035_PS_TRIG
{
    PS_TRIG_LOW = 0,
    PS_TRIG_HIGH = 1,
} VCNL4035_PS_TRIG;

/* Proximity Sensor Active Force Mode Enable */
typedef enum VCNL4035_PS_AF
{
    PS_AF_OFF = 0,
    PS_AF_ON = 1,
} VCNL4035_PS_AF;

/* Proximity Sensor Smart Persistance Mode Enable */
typedef enum VCNL4035_PS_SMART_PERS
{
    PS_SMART_PERS_OFF = 0,
    PS_SMART_PERS_ON = 1,
} VCNL4035_PS_SMART_PERS;

/* Proximity Sensor IRED Selection */
typedef enum VCNL4035_IRED_SELECT
{
    IRED_SELECT_1 = 0,
    IRED_SELECT_2 = 1,
    IRED_SELECT_3 = 2,
} VCNL4035_IRED_SELECT;

/* Proximity Sensor Smart LED Low Current Mode: 
Reduces the LED current [Amps] by a factor of 10. */
typedef enum VCNL4035_LED_I_LOW
{
    LED_I_LOW_OFF = 0,
    LED_I_LOW_ON = 1,
} VCNL4035_PS_LED_I_LOW;

/* PS MS */

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

/* Proximity Sensor Sunlight Protect Output */
typedef enum VCNL4035_PS_SPO
{
    PS_SPO_00H = 0,
    PS_SPO_FFH = 1,
} VCNL4035_PS_SPO;

/* Proximity Sensor Sunlight Protect Capability */
typedef enum VCNL4035_PS_SP
{
    PS_SP_TYPICAL_1 = 0,   // Typical sunlight capability
    PS_SP_TYPICAL_1_5 = 1, // 1.5 x Typical sunlight capability
} VCNL4035_PS_SP;

/* Proximity Sensor Sunlight Cancel Current */
typedef enum VCNL4035_PS_SC_CUR
{
    PS_SC_CUR_TYPICAL_TIMES_1 = 0,
    PS_SC_CUR_TYPICAL_TIMES_2 = 1,
    PS_SC_CUR_TYPICAL_TIMES_4 = 2,
    PS_SC_CUR_TYPICAL_TIMES_8 = 3,
} VCNL4035_PS_SC_CUR;

/* Structure Defintions */
typedef struct ps_conf_1_s
{
    VCNL4035_PS_SD ps_sd;
    VCNL4035_PS_IT ps_it;
    VCNL4035_PS_PERS ps_pers;
    VCNL4035_PS_DUTY ps_duty;
} ps_conf_1_t;

typedef struct ps_conf_2_s
{
    VCNL4035_PS_INT ps_int;
    VCNL4035_PS_NS ps_ns;
    VCNL4035_PS_HD ps_hd;
    VCNL4035_PS_GAIN ps_gain;
    VCNL4035_GESTURE_MODE gesture_mode;
    VCNL4035_GESTURE_INT_EN gesture_int_en;
} ps_conf_2_t;

typedef struct ps_conf_3_s
{
    VCNL4035_PS_SC_EN ps_sc_en;
    VCNL4035_PS_MS_COMMAND ps_ms;
    VCNL4035_PS_TRIG ps_trig;
    VCNL4035_PS_AF ps_af;
    VCNL4035_PS_SMART_PERS ps_smart_pers;
    VCNL4035_IRED_SELECT ired_select;
    VCNL4035_LED_I_LOW led_i_low;
} ps_conf_3_t;

typedef struct ps_ms_s
{
    VCNL4035_LED_I led_i;
    VCNL4035_PS_SPO ps_spo;
    VCNL4035_PS_SP ps_sp;
    VCNL4035_PS_SC_CUR ps_sc_cur;
} ps_ms_t;

typedef struct ps_settings_s
{
    ps_conf_1_t ps_conf_1;
    ps_conf_2_t ps_conf_2;
    ps_conf_3_t ps_conf_3;
    ps_ms_t ps_ms;
    uint16_t ps_thdl;
    uint16_t ps_thdh;
} ps_settings_t;

#endif /* VCNL4035_Defs_H */