/****************************************************************
Set_Register_Test.ino
VCNL4035X01 IC
Luke Young
13/05/2019
https://github.com/LukeYoung3000/Vishay_VCNL4035_Arduino

Shows how to set the proximity integration time and led current of the
VCNL4035. 

Tests the "setPsIntegrationTime" and "setLedCurrent" functions from VCNL4035_Lib
to change the input parameters to these functions see VCNL4035_Lib.h in the
enum definitions for more valid options.

Hardware Connections:
 
 Arduino Pin  VCNL4035        Function
 ---------------------------------------
 3.3V         VCC(Pin 1)      Power
 GND          GND(Pin 3)      Ground
 A4(SDA)      SDA(Pin 8)      I2C Data
 A5(SCL)      SCL(Pin 2)      I2C Clock

Resources:
Include Wire.h and VCNL4035_Lib.h

Development environment specifics:
Written in Arduino 1.8.5
Tested with a Arduino UNO

This code is beerware; if you see me at the local,
and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/

#include <VCNL4035_Lib.h>

// Create VCNL4035 object
VCNL4035 vcnl;

void setup()
{
  vcnl.init(GESTURE);

  Serial.begin(115200);

  Serial.println("SET PS_IT TEST: ");

  // Change the Integration time to 400us (8T)
  vcnl.setPsIntegrationTime(PS_IT_400us);
  delay(100);
  // Change the Integration time to 125us (2.5T)
  vcnl.setPsIntegrationTime(PS_IT_125us);

  Serial.println("SET LED_I TEST: ");

  // Change the LED current to 180mA
  vcnl.setLedCurrent(LED_I_180_mA);
  delay(100);
  // Change the LED current to 75mA
  vcnl.setLedCurrent(LED_I_75_mA);
}

void loop()
{
  delay(1000);
}
