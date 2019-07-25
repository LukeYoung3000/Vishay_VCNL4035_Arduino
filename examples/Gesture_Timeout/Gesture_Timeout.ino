/*
This code:
*/

#define VCNL_INT_PIN 2

#include <VCNL4035_App.h>
#include <Handy_Arduino_Lib.h>

VCNL4035_Application vcnl;
uint16_t time_limit_ms = 5000;
Timer_ms power_save_timer(time_limit_ms);

uint16_t sens_data[3] = {0};

void setup()
{
  /* Hardware Settings */
  Serial.begin(115200);
  pinMode(VCNL_INT_PIN, INPUT_PULLUP);

  /* VCNL Settings Public Variable Settings*/
  vcnl.gesture_mode.ps_conf_1.ps_it = PS_IT_400us;
  vcnl.ps_mode.ps_conf_1.ps_it = PS_IT_400us;
  vcnl.ps_mode.ps_thdl = 50;
  vcnl.ps_mode.ps_thdh = 300;
  /* VCNL Settings */
  vcnl.setGestureActiveThreshold(500);
  vcnl.setIntPin(VCNL_INT_PIN);
  vcnl.setGestureNoiseSubtract(true);
  vcnl.init(GESTURE);
  vcnl.calcGestureNoise(10);

  /* Reset SW Timmer */
  power_save_timer.reset_timer();
}

void loop()
{
  if (vcnl.isGestureActivity())
  {
    power_save_timer.reset_timer();
  }

  if (power_save_timer.check_overflow())
  {
    vcnl.setSleepMode();
  }

  vcnl.MainLoop();

  if (vcnl.isDataReady())
  {
    vcnl.getSensorData(sens_data);
    Serial.print(sens_data[0]);
    Serial.print(", ");
    Serial.print(sens_data[1]);
    Serial.print(", ");
    Serial.println(sens_data[2]);
  }
}
