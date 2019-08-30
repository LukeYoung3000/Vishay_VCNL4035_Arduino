#define INTERRUPT_PIN 2

#include <VCNL4035_App.h>

// Create VCNL4035 object
VCNL4035_Application vcnl;

// Variable to store proximity data from all three sensors
int16_t x = 0;
int16_t z = 0;

void setup()
{
    pinMode(INTERRUPT_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    vcnl.setIntPin(INTERRUPT_PIN);
    vcnl.init(GESTURE);
    vcnl.setPsIntegrationTime(PS_IT_400us);
    vcnl.setLedCurrent(LED_I_100_mA);
    vcnl.calcGestureNoise(100);
    vcnl.setGestureNoiseSubtract(true);
    // Add ZX enable function
    vcnl.gesture_flags.zx_enable = 1;
}

void loop()
{
    vcnl.MainLoop();

    if (vcnl.isZXDataReady())
    {
        vcnl.getZXData(&z, &x);
        Serial.print(z);
        Serial.print(" , ");
        Serial.println(x);
    }
}
