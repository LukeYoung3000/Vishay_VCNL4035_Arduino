#define INTERRUPT_PIN 2

#include <VCNL4035_App.h>

// Create VCNL4035 object
VCNL4035_Application vcnl;

// Variable to store proximity data from all three sensors
int16_t x_pos = 0;
int16_t z_pos = 0;
uint16_t sens_data[3];

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
    vcnl.gesture_flags.zx_enable = 1;
    vcnl.gesture_flags.zx_smooth_enable = 0;
    vcnl.gesture_flags.orb_detect_enable = 1;
}

void loop()
{
    vcnl.MainLoop();

    if (vcnl.isOrbDataReady())
    {
        vcnl.getZXData(&z_pos, &x_pos);
        Serial.print(z_pos);
        Serial.print(" , ");
        Serial.print(x_pos);
        Serial.print(" , ");
        vcnl.getSensorData(sens_data);
        Serial.println(sens_data[1]);
    }
}