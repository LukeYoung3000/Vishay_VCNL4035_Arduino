#include <VCNL4035_Lib.h>

// Create VCNL4035 object
VCNL4035 vcnl;
uint16_t proxy_data[3] = {0};

void setup()
{
    // Set up VCNL registers
    vcnl.init(PROXIMITY_SENSOR);
    // Change the Integration time to the largest possible time (8T)
    vcnl.setPsIntegrationTime(PS_IT_400us);
    Serial.begin(115200);
}

void loop()
{
    delay(50);
    vcnl.readGestureData(proxy_data);

    // Print Data from IRED1:
    Serial.println(proxy_data[0]);
}