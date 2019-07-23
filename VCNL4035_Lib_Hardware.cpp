#include <Arduino.h>
#include <Wire.h>

#include "VCNL4035_Lib.h"

/*******************************************************************************
 * Raw I2C Reads and Writes
 * Taken and Modified from the ZX_Sensor library from Spark-fun
 ******************************************************************************/

void VCNL4035::i2cBegin()
{
    Wire.begin();
}

/**
 * @brief Writes a single byte to the I2C device (no register)
 *
 * @param[in] val the 1-byte value to write to the I2C device
 * @return True if successful write operation. False otherwise.
 */
bool VCNL4035::i2cWriteByte(uint8_t val)
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
bool VCNL4035::i2cWriteReg(uint8_t reg, uint8_t low_byte, uint8_t high_byte)
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
bool VCNL4035::i2cReadReg(uint8_t reg, uint16_t &val)
{
    uint16_t bytes[2] = {0, 0};
    uint8_t i = 0;

    /* Indicate which register we want to read from */
    if (!i2cWriteByte(reg))
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
bool VCNL4035::i2cWriteRegBits(uint8_t reg, uint8_t msb_lsb,
                               uint8_t start_bit, uint8_t end_bit, uint8_t val)
{
    /* Check for valid bit range */
    int8_t num_bits = end_bit - start_bit;
    if (num_bits < 0)
        return false;

    /* Read current value from register */
    uint16_t current_val = 0;
    if (!i2cReadReg(reg, current_val))
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
    if (!i2cWriteReg(reg, low, high))
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
bool VCNL4035::i2cReadPsData(uint16_t *val)
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