/* This code prints data from ps sensor and toggles the onboard led when an
gets to close to the sensor */

#define INTERRUPT_PIN 2

#include <VCNL4035_Lib.h>

// Create VCNL4035 object
VCNL4035 vcnl;
// Low and High proximity threshold values
uint16_t threshold_low = 100;
uint16_t threshold_high = 500;
// Variable to store proximity data from all three sensors
uint16_t proxy_data[3] = {0};

void setup()
{

    Serial.begin(115200);
    pinMode(13, OUTPUT); // Onboard LED

    // Set up VCNL registers
    vcnl.init(PROXIMITY_SENSOR);
    // Change the Integration time to the largest possible time (8T)
    vcnl.setPsIntegrationTime(PS_IT_400us);
    // Set the low and high threshold values for the interrupt
    vcnl.setPsInterruptThresholdLow(threshold_low);
    vcnl.setPsInterruptThresholdHigh(threshold_high);

    // Set up an interrupt on Arduino Pin 2
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), ISR_vcnl, FALLING);

    delay(10);
}

void loop()
{
    delay(50);
    // Read proximity data from VCNL
    vcnl.readPsData(proxy_data);
    // Print data
    Serial.println(proxy_data[0]);
}

void ISR_vcnl()
{
    // Toggle LED
    digitalWrite(13, !digitalRead(13));

    // Turn interrupts on for I2C
    interrupts();

    // Read interrupt flag on VCNL (Sets VCNL INT pin back to high)
    vcnl.readInterruptFlags();
}