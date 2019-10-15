/*
The following code determines left and right swipe gestures useing data from the VCNL4035. It should be ran on the Arduino MEGA as there are large float arrays used during the convolution stage of the algorithm. This code will be used to design the gesture sensor section in VCNL4035_Application library.
*/

#include <VCNL4035_App.h>
#include <myStats_lib.h>
#include <Handy_Arduino_Lib.h>

#define VCNL_INTERRUPT_PIN 2

// VCNL Object
VCNL4035_Application vcnl;

// Stats Global Varribles
myStats stat_left;
myStats stat_right;
uint16_t gesture_state_machine = 0;
int cnt = 1;
uint16_t left_buf[GESTURE_BUF_SIZE] = {0};
uint16_t right_buf[GESTURE_BUF_SIZE] = {0};
int i_left = 0;
int i_right = 0;
// Timer Object
Timer_ms gesture_timeout(1000);

// Stats Parameters
int sample_skip = 2;
uint16_t std_trigger = 25;

// Sensor Data Array
uint16_t sens_data[3] = {0};

void setup()
{
    pinMode(VCNL_INTERRUPT_PIN, INPUT_PULLUP);
    Serial.begin(115200);

    vcnl.setIntPin(VCNL_INTERRUPT_PIN);
    vcnl.init(GESTURE);
    vcnl.setPsIntegrationTime(PS_IT_400us);
    vcnl.setLedCurrent(LED_I_100_mA);
    vcnl.calcGestureNoise(100);
    vcnl.setGestureNoiseSubtract(true);

    // Print Gesture Noise Calculations
    if (0)
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

        if (gesture_state_machine == 0)
        {
            if (cnt++ >= sample_skip)
            {
                // Standard Deviation Uint Point Calculation:
                uint16_t left_std;
                left_std = stat_left.Standard_deviation(sens_data[0]);
                uint16_t right_std;
                right_std = stat_right.Standard_deviation(sens_data[2]);

                // Serial Print Standard Deviation:
                Serial.print(left_std);
                Serial.print(" , ");
                Serial.print(right_std);
                Serial.println();

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
                left_buf[i_left++] = sens_data[0];
                right_buf[i_right++] = sens_data[2];
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

            // After Calculations:
            Serial.println("-------------------------------------");
            Serial.print("Delay Is : ");
            Serial.println(delay);
            Serial.print("XCorr Time is: ");
            Serial.print(after_time - prev_time);
            Serial.println("-------------------------------------");

            // Set Timer:
            gesture_timeout.reset_timer();
            gesture_state_machine = 4;
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
    }
}
