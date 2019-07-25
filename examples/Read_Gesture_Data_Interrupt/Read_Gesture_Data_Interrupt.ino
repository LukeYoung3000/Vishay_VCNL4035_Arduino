#define INTERRUPT_PIN 2

#include <VCNL4035_Lib.h>

// Create VCNL4035 object
VCNL4035 vcnl;

// Variable to store proximity data from all three sensors
uint16_t proxy_data[3] = {0};

void setup()
{

    Serial.begin(115200);

    // Set up VCNL registers
    vcnl.init(GESTURE);
    // Change the Integration time to the largest possible time (8T)
    vcnl.setPsIntegrationTime(PS_IT_400us);

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
    vcnl.readGestureData(proxy_data);

    // Print Data:
    Serial.print(proxy_data[0]);
    Serial.print(" , ");
    Serial.print(proxy_data[1]);
    Serial.print(" , ");
    Serial.println(proxy_data[2]);

    // Clear vcnl interrupt register
    vcnl.readInterruptFlags();

    // Reset gesture trigger:
    vcnl.setPsTrigger();
}
