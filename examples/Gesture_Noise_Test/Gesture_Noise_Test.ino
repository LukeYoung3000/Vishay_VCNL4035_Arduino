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
    vcnl.calcGestureNoise(10);

    Serial.println(" VCNL4035_Application::calcGestureNoise");
    Serial.print(vcnl.gesture_noise[0]);
    Serial.print(", ");
    Serial.print(vcnl.gesture_noise[1]);
    Serial.print(", ");
    Serial.println(vcnl.gesture_noise[2]);
}

void loop()
{
}