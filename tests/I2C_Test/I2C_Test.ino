/*
The following program tests I2C coms from the VCNL4035 useing the wire libray.
The "vishay_readData" file shows two methods for reading in data.
The methods functions in different ways for different microcontrollers.
*/

#include <Wire.h>

// Uncomment line bellow when using the Arduino Due's 2nd I2C
// #define Wire Wire1

#define VCNL_INT_PIN 2

//I2C Address
#define VISHAY_ADDR 0x60

//I2C Register Addresses
#define PS_CONF_1_2 0x3
#define PS_CONF_3_MS 0x4
#define RES_INT_FLAGS 0xD
#define PS1_DATA_REG 0x8
#define PS2_DATA_REG 0x9
#define PS3_DATA_REG 0xA
#define VISHAY_VERSION_REG 0xE

// PS_CONF_1:
// Turn on PS (Bit 0 = 0)
// Set Intergration Time to 4T (Bit 3:2:1 = 1:1:0)
// uint8_t ps_conf_1 = 0x0c;
//  8T:
uint8_t ps_conf_1 = 0x0e;
//  1T:
// uint8_t ps_conf_1 = 0x00;

// PS_CONF_2:
// Turn on GESTURE_INT_EN (Bit 7 = 1)
// Turn on GESTURE_MODE (Bit 6 = 1), Set PS_HD to 16bit (Bit 3 = 1), Set PS_Gain to two step mode (Bit 5:4 = 0:0)
uint8_t ps_conf_2 = 0xc4;
//PS_CONF_3: Turn on PS_AF (active force mode) (Bit 3 = 1)
uint8_t ps_conf_3 = 0x08;
//PS_MS: Set LED Current (LED_I) to 100mA (Bit 0:1:2 = 0:1:0)
uint8_t ps_ms = 0x02;

// Raw data array
uint16_t ps[3] = {0};

void setup()
{
  pinMode(VCNL_INT_PIN, INPUT_PULLUP);
  Serial.begin(115200);

  Wire.begin();

  delay(500);

  vishay_write16_LowHigh(PS_CONF_1_2, ps_conf_1, ps_conf_2);
  vishay_write16_LowHigh(PS_CONF_3_MS, ps_conf_3, ps_ms);

  // Set Threshold Values
  vishay_write16_LowHigh(0x06, 0, 0);
  vishay_write16_LowHigh(0x07, 0xff, 0xff);

  delay(200);

  /* Set Trigger */
  ps_conf_3 = 0x0c;
  vishay_write16_LowHigh(PS_CONF_3_MS, ps_conf_3, ps_ms);
}

void loop()
{
  if (digitalRead(VCNL_INT_PIN) == 0)
  {

    // Print Data
    uint16_t ps1 = 0; // Left IRED
    uint16_t ps2 = 0; // Right IRED
    uint16_t ps3 = 0; // Center IRED

    ps1 = vishay_readData(PS1_DATA_REG); // Left
    ps2 = vishay_readData(PS2_DATA_REG); // Right
    ps3 = vishay_readData(PS3_DATA_REG); // Center

    Serial.print(ps1);
    Serial.print(", ");
    Serial.print(ps2);
    Serial.print(", ");
    Serial.println(ps3);

    //vishay_readPsData(ps);
    //Serial.print(ps[0]); Serial.print(", ");
    //Serial.print(ps[1]); Serial.print(", ");
    //Serial.println(ps[2]);

    /* Reset Trigger */
    vishay_write16_LowHigh(PS_CONF_3_MS, ps_conf_3, ps_ms);
  }
  delay(10);
}

void vishay_write16_LowHigh(uint8_t address, uint8_t low, uint8_t high)
{
  Wire.beginTransmission(VISHAY_ADDR);
  Wire.write(address);
  Wire.write(low);
  Wire.write(high);
  Wire.endTransmission();
}

uint16_t vishay_readData(uint8_t command_code)
{
  uint16_t reading;
  uint16_t byteLow = 0;
  uint16_t byteHigh = 0;
  uint16_t bytes[2] = {0, 0};
  int i = 0;
  Wire.beginTransmission(VISHAY_ADDR);
  Wire.write(command_code);
  Wire.endTransmission(uint8_t(false));
  Wire.requestFrom((uint8_t)VISHAY_ADDR, uint8_t(2));

  // Read from register (Method 1)
  while (!Wire.available())
  {
  }
  byteLow = Wire.read();
  while (!Wire.available())
  {
  }
  byteHigh = Wire.read();

  /*
  // Read from register (Method 2)
  while (Wire.available())
  {
     // Possible over flow errors here
     bytes[i] = Wire.read();
     i++;
  }
  bytes[0] = byteLow;
  bytes[1] = byteHigh;
  */
  reading = (byteHigh <<= 8) + byteLow;
  return reading;
}

void vishay_readPsData(uint16_t *raw_data_buffer)
{
  uint16_t reading;
  Wire.beginTransmission(VISHAY_ADDR);
  Wire.write(PS1_DATA_REG);
  Wire.endTransmission(false);
  Wire.requestFrom((uint8_t)VISHAY_ADDR, uint8_t(6));
  for (int i = 0; i < 3; i++)
  {
    while (!Wire.available())
      ;
    uint8_t byteLow = Wire.read();
    while (!Wire.available())
      ;
    uint16_t byteHigh = Wire.read();
    reading = (byteHigh <<= 8) + byteLow;
    raw_data_buffer[i] = reading;
  }
}

/*
uint16_t vishay_readData2(uint8_t command_code)
{
  uint16_t reading;
  //Wire.beginTransmission(VISHAY_ADDR);
  Wire.requestFrom((uint8_t) VISHAY_ADDR, (uint8_t) 1, (uint32_t) command_code, (uint8_t) 1);
  while (!Wire.available());
  uint8_t byteLow = Wire.read();
  while (!Wire.available());
  uint16_t byteHigh = Wire.read();
  reading = (byteHigh <<= 8) + byteLow;
  //Wire.endTransmission();
  return reading;
}
*/
