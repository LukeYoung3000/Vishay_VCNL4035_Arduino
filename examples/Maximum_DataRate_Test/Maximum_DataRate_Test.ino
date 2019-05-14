/****************************************************************
Maximum_DataRate_Test.ino
VCNL4035X01 IC
Luke Young
13/05/2019
https://github.com/LukeYoung3000/Vishay_VCNL4035_Arduino

Tests the fastest data rate possible using gesture mode on the VCNL4035.
The program uses an ISR to read in data from the VCNL and set a trigger
to initiate the collection of the next set of sensor data. Put an oscilloscope
probe between GRD and the VCNL4035 INT Pin to determine the maximum data rate.

Hardware Connections:
 
 Arduino Pin  VCNL4035        Function
 ---------------------------------------
 3.3V         VCC(Pin 1)      Power
 GND          GND(Pin 3)      Ground
 A4(SDA)      SDA(Pin 8)      I2C Data
 A5(SCL)      SCL(Pin 2)      I2C Clock
 2            INT(Pin 7)      Data Ready

Resources:
Include Wire.h and VCNL4035.h

Development environment specifics:
Written in Arduino 1.8.5
Tested with a Arduino UNO

This code is beerware; if you see me at the local,
and you've found our code helpful, please
buy us a round!

Distributed as-is; no warranty is given.
****************************************************************/

#define INTERRUPT_PIN 2

#include <VCNL4035_Lib.h>

// Create VCNL4035 object
VCNL4035 vcnl;

// Variable to store proximity data from all three sensors
uint16_t proxy_data[3] = {0};

int cnt = 0;

void setup()
{

  Serial.begin(115200);

  // Set up VCNL registers
  vcnl.init();
  // Change the Integration time to the smallest possible time (1T)
  vcnl.setPsIntegrationTime(PS_IT_1T);

  // Set up an interrupt on Arduino Pin 2 (VCNL INT pin goes low when data is ready)
  pinMode(INTERRUPT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), ISR_vcnl, FALLING);

  // Read interrupt flag on VCNL (Sets VCNL INT pin high)
  vcnl.readInterruptFlags();
  delay(10);

  // Trigger gesture data capture
  vcnl.setPsTrigger();
}

void loop()
{
  // Put something in the main loop
  delay(50);
}

void ISR_vcnl()
{
  // Turn interrupts on for I2C
  interrupts();

  // Read Regs:
  vcnl.readPsData(proxy_data);

  // Print Data:
  /*
  Serial.print(proxy_data[0]); Serial.print(", ");
  Serial.print(proxy_data[1]); Serial.print(", ");
  Serial.println(proxy_data[2]);
  */
  
  // Print Time:
  /*
  if(cnt++ >= 999)
  {
    cnt = 0;
    Serial.println(millis());
  }
  */
  
  // Clear vcnl interrupt register:
  vcnl.readInterruptFlags();

  // Reset gesture trigger:
  vcnl.setPsTrigger();
}
