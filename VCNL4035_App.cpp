/**
 * @file    VCNL4035_App.cpp
 * @brief   Library for the VCNL4035 - Application Specific
 * @author  Luke Young (L S Young Electrical)
 *
 * @copyright	This code is public domain but you buy me a beer if you use
 * this and we meet someday (Beerware license).
 * 
 * The libray provides an application specific class for the VCNL4035. It
 * requires the parent class "VCNL4035" which can be found in "VCNL4035_Lib.h".
 * It includes algorithms to detect varrios swiping gestures, ZX position above
 * the sensor and powersaveing sleep mode functionality.  
 */

#include <Arduino.h>

#include "VCNL4035_App.h"

/* Operational */

void VCNL4035_Application::MainLoop()
{

    if (current_mode != prev_mode)
    {
        if (current_mode == ACTIVE)
        {
            setGestureMode();
            setPsTrigger();
        }
        else if (current_mode == SLEEP)
        {
            setPSMode();
            readInterruptFlags();
        }
        prev_mode = current_mode;
    }

    //Serial.println("current_mode:");
    //Serial.println(current_mode);
    //Serial.println("ACTIVE:");
    //Serial.println(ACTIVE);

    if (current_mode == ACTIVE)
    {
        ActiveLoop();
    }
    else if (current_mode == SLEEP)
        SleepLoop();
}

void VCNL4035_Application::ActiveLoop()
{
    if (readInt() == 0)
    {
        // Read in raw data
        readGestureData(sensor_data);

        // Subtract noise average noise if enabled
        if (gesture_flags.noise_canc_enable)
        {
            subtractGestureNoise(sensor_data);
        }

        // Check if data is in the active range
        bool a = sensor_data[0] > gesture_activity_threshold;
        bool b = sensor_data[1] > gesture_activity_threshold;
        bool c = sensor_data[2] > gesture_activity_threshold;
        if (a || b || c)
            gesture_flags.activity_status = 1;
        else
            gesture_flags.activity_status = 0;

        // Calculate ZX cordinate
        if (gesture_flags.zx_enable)
        {
            // Use data to calculate ZX ...
        }

        // Detect valid gestures
        if (gesture_flags.gesture_recognition_enable)
        {
            // Use data to calculate if a gesture has occurred ...
        }

        // Set data ready flag
        gesture_flags.data_ready = 1;
        //Serial.println("In Actiive loop: data_ready_flag");
        //Serial.println(gesture_flags.data_ready);

        // Restart the data collection process
        readInterruptFlags();
        setPsTrigger();
    }
}

void VCNL4035_Application::SleepLoop()
{
    if (readInt() == 0)
    {
        // Wake up and Change to active mode
        setActiveMode();
    }
}

void VCNL4035_Application::calcGestureNoise(uint8_t number_of_readings)
{
    if (number_of_readings > 0)
    {
        uint16_t data[3];

        for (uint8_t i = 0; i < number_of_readings; i++)
        {
            readInterruptFlags();
            setPsTrigger();
            while (readInt())
            {
            }
            readGestureData(data);
            gesture_noise[0] = gesture_noise[0] + data[0] / number_of_readings;
            gesture_noise[1] = gesture_noise[1] + data[1] / number_of_readings;
            gesture_noise[2] = gesture_noise[2] + data[2] / number_of_readings;
        }
    }
}

void VCNL4035_Application::subtractGestureNoise(uint16_t *data)
{
    for (uint8_t i = 0; i < 3; i++)
    {
        if (gesture_noise[i] >= data[i])
        {
            data[i] = 0;
        }
        else
        {
            data[i] = data[i] - gesture_noise[i];
        }
    }
}

/* Checks */
bool VCNL4035_Application::isDataReady()
{
    if (gesture_flags.data_ready)
    {
        gesture_flags.data_ready = 0;
        return (1);
    }
    return (0);
}

bool VCNL4035_Application::isGestureActivity()
{
    if (gesture_flags.activity_status)
        return (true);
    else
        return (false);
}

/* Gets */
void VCNL4035_Application::getSensorData(uint16_t *data)
{
    data[0] = sensor_data[0];
    data[1] = sensor_data[1];
    data[2] = sensor_data[2];
}

void VCNL4035_Application::getZXData(uint16_t *Z, uint16_t *X)
{
    *Z = Z_value;
    *X = X_value;
}

/* Settings */
void VCNL4035_Application::setGestureNoiseSubtract(uint8_t state)
{
    if (state > 0)
        gesture_flags.noise_canc_enable = 1;
    else
        gesture_flags.noise_canc_enable = 0;
}

void VCNL4035_Application::setGestureActiveThreshold(uint16_t threshold)
{
    gesture_activity_threshold = threshold;
}

void VCNL4035_Application::setSleepMode()
{
    current_mode = SLEEP;
}

void VCNL4035_Application::setActiveMode()
{
    current_mode = ACTIVE;
}
