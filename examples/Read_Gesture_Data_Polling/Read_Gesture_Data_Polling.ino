#define INTERRUPT_PIN 2

#include <VCNL4035_App.h>

// Create VCNL4035 object
VCNL4035_Application vcnl;

// Variable to store proximity data from all three sensors
uint16_t sens_data[3] = {0};

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

    // Print Gesture Noise Calculations
    if (1)
    {
        Serial.println("Gesture Noise:");
        Serial.print(vcnl.gesture_noise_LDR1);
        Serial.print(" , ");
        Serial.print(vcnl.gesture_noise_LDR2);
        Serial.print(" , ");
        Serial.println(vcnl.gesture_noise_LDR3);
    }
}

void loop()
{
    vcnl.MainLoop();

    if (vcnl.isDataReady())
    {
        vcnl.getSensorData(sens_data);
        Serial.print(sens_data[0]);
        Serial.print(" , ");
        Serial.print(sens_data[1]);
        Serial.print(" , ");
        Serial.println(sens_data[2]);
    }
}