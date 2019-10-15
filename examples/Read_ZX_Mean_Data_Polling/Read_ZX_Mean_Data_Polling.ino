#define INTERRUPT_PIN 2

#include <VCNL4035_App.h>

// Create VCNL4035 object
VCNL4035_Application vcnl;

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
    vcnl.gesture_flags.zx_smooth_enable = 1;
}

void loop()
{
    vcnl.MainLoop();

    if (vcnl.isZXDataReady())
    {
        Serial.print(vcnl.Z_value);
        Serial.print(" , ");
        Serial.println(vcnl.X_value);
    }
}
