/*
This code:
 - Reads gesture data from the VCNL4035 and prints to serial.
 - Turns Pin 13 LED OFF when valid data has not been detected for 5 seconds.
 - Turns Pin 13 LED ON while valid data is detected.
*/

#define VCNL_INT_PIN 2

#include <VCNL4035_Lib.h>

// Create VCNL4035 object
VCNL4035 vcnl;

// Timer Class
class Timer_ms
{
private:
  uint16_t time_limit;
  unsigned long time_start;
  boolean overflow_flag;

public:
  Timer_ms(uint16_t Time_limit);
  boolean check_overflow();
  void reset_timer();
};

// Variables
uint16_t time_limit_ms = 5000;
Timer_ms timer(time_limit_ms);

uint16_t proxy_data[3] = {0};
uint16_t activity_threshold = 100;
uint8_t active_mode = 1;
uint8_t sleep_mode = 2;
uint8_t prev_mode;
uint8_t current_mode;

void setup()
{
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  Serial.begin(115200);

  prev_mode = active_mode;
  current_mode = active_mode;

  // Set up VCNL registers
  vcnl.init(GESTURE);
  // Change the Integration time to the largest possible time (8T)
  vcnl.setPsIntegrationTime(PS_IT_400us);

  // Read interrupt flag on VCNL (Sets VCNL INT pin high)
  vcnl.readInterruptFlags();
  delay(10);

  // Trigger gesture data capture
  vcnl.setPsTrigger();
}

void loop()
{
  if (!digitalRead(VCNL_INT_PIN))
  {
    // Read Regs:
    vcnl.readPsData(proxy_data);
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
    // Check if the sensor is activity being used
    if (proxy_data[0] > activity_threshold)
    {
      current_mode = active_mode;
      // Reset active timer
      timer.reset_timer();
    }
  }

  if (timer.check_overflow())
  {
    current_mode = sleep_mode;
  }

  if (prev_mode != current_mode)
  {
    if (current_mode == active_mode)
      digitalWrite(13, HIGH);
    else
      digitalWrite(13, LOW);
  }
  prev_mode = current_mode;
}

/* Timer_ms Class Methods */

// Constructor
Timer_ms::Timer_ms(uint16_t Time_limit)
{
  time_limit = Time_limit;
  overflow_flag = 0;
  time_start = 0;
}
// Check Timer Overflow
boolean Timer_ms::check_overflow()
{
  if (overflow_flag == 0)
  {
    if (millis() > (time_start + time_limit))
    {
      overflow_flag = 1;
      return 1;
    }
  }
  return 0;
}
// Reset Timer
void Timer_ms::reset_timer()
{
  time_start = millis();
  overflow_flag = 0;
}