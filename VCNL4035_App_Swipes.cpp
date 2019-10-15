#include "VCNL4035_App.h"
#include <myStats_lib.h>
#include <Handy_Arduino_Lib.h>

#define VCNL4035_APP_SWIPES_DEBUG 0

/* Gesture Recognition Code */

// Internal Varribles
myStats stat_left;
myStats stat_right;
uint16_t gesture_state_machine = 0;
int cnt = 1;
uint16_t left_buf[GESTURE_BUF_SIZE] = {0};
uint16_t right_buf[GESTURE_BUF_SIZE] = {0};
int i_left = 0;
int i_right = 0;
Timer_ms gesture_timeout(1000);

// Returns a 1 when new gesture results data are ready
uint8_t VCNL4035_Application::calcGestureRecognition()
{
    if (gesture_state_machine == 0)
    {
        if (cnt++ >= sample_skip)
        {
            // Standard Deviation Uint Point Calculation:
            uint16_t left_std;
            left_std = stat_left.Standard_deviation(sensor_data[0]);
            uint16_t right_std;
            right_std = stat_right.Standard_deviation(sensor_data[2]);

#if VCNL4035_APP_SWIPES_DEBUG
            //Serial Print Standard Deviation:
            Serial.print(left_std);
            Serial.print(" , ");
            Serial.print(right_std);
            Serial.println();
#endif

            // Reset Count:
            cnt = 1;
            // Check For Gesture:
            if ((left_std > std_trigger) && (right_std > std_trigger))
            {
                gesture_state_machine = 1;
            }
        }
    }
    // Load Existing Data Points Into a Buffer:
    if (gesture_state_machine == 1)
    {
        i_left = 0;
        while (!stat_left.statsBuffer.isEmpty())
        {
            left_buf[i_left++] = stat_left.statsBuffer.shift();
        }
        i_right = 0;
        while (!stat_right.statsBuffer.isEmpty())
        {
            right_buf[i_right++] = stat_right.statsBuffer.shift();
        }
        stat_left.Reset_frame();
        stat_right.Reset_frame();
        if (i_left != i_right)
        {
            // Jump to Delay Stage, Do Not Calculate XCorr
            gesture_timeout.reset_timer();
            gesture_state_machine = 4;
        }
        else
        {
            i_left--;
            i_right--;
            // Continue To Next Stage
            gesture_state_machine = 2;
        }
    }
    // Fill Buffer With New Data
    if (gesture_state_machine == 2)
    {
        if (cnt++ >= sample_skip)
        {
            left_buf[i_left++] = sensor_data[0];
            right_buf[i_right++] = sensor_data[2];
            cnt = 1;
            // When Buffer Full:
            if (i_left >= GESTURE_BUF_SIZE || i_right >= GESTURE_BUF_SIZE)
            {
                gesture_state_machine = 3;
            }
        }
    }
    // Cross Corelation Calculations:
    if (gesture_state_machine == 3)
    {
        int delay = 999;
        uint32_t prev_time = millis();
        delay = myStats::Cross_correlation(left_buf, right_buf, (int)GESTURE_BUF_SIZE);
        uint32_t after_time = millis();

#if VCNL4035_APP_SWIPES_DEBUG
        // Serial Print Results:
        Serial.println("-------------------------------------");
        Serial.print("Delay Is : ");
        Serial.println(delay);
        Serial.print("XCorr Time is: ");
        Serial.print(after_time - prev_time);
        Serial.println("-------------------------------------");
#endif

        // Set Timer:
        gesture_timeout.reset_timer();
        gesture_state_machine = 4;
        // return 1 because delay has been calculated
        swipe_speed = delay;
        return 1;
    }
    // Delay Before Restart Gesture Mode:
    if (gesture_state_machine == 4)
    {
        if (gesture_timeout.check_overflow())
        {
            // Reset Gesture Mode:
            gesture_state_machine = 0;
        }
    }
    return 0;
}

/* Moving Mean Code */
myStats mean_left;
myStats mean_right;

void VCNL4035_Application::calcMovingMean(uint16_t left, uint16_t right)
{
    mean_left_value = (uint16_t)mean_left.Mean(left);
    mean_right_value = (uint16_t)mean_right.Mean(right);
}
