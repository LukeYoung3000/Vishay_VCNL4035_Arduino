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
#include <math.h>

#include "VCNL4035_App.h"

/* Constructor */

VCNL4035_Application::VCNL4035_Application()
{
    /* Inital Variable Settings */
    gesture_noise_LDR1 = 0;
    gesture_noise_LDR2 = 0;
    gesture_noise_LDR3 = 0;
    gesture_flags = {0};
    current_mode = ACTIVE;
    prev_mode = ACTIVE;
    gesture_activity_threshold = 100;
    sensor_data[0] = 0;
    sensor_data[1] = 0;
    sensor_data[2] = 0;
    Z_value = 0;
    X_value = 0;
    ZX_min_threshold = 22;
}

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
            if (calcZXData())
                gesture_flags.zx_data_ready = 1;
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
        float data_f0 = 0;
        float data_f1 = 0;
        float data_f2 = 0;
        float gn_f0 = 0;
        float gn_f1 = 0;
        float gn_f2 = 0;

        for (uint8_t i = 0; i < number_of_readings; i++)
        {
            readInterruptFlags();
            setPsTrigger();
            while (readInt())
            {
                // Arduino Function - Should use a proper timer
                delay(1);
            }
            readGestureData(data);
            data_f0 = (float)data[0];
            data_f1 = (float)data[1];
            data_f2 = (float)data[2];

            gn_f0 += data_f0 / (float)number_of_readings;
            gn_f1 += data_f1 / (float)number_of_readings;
            gn_f2 += data_f2 / (float)number_of_readings;
        }
        gesture_noise_LDR1 = gn_f0;
        gesture_noise_LDR2 = gn_f1;
        gesture_noise_LDR3 = gn_f2;
    }
}

uint8_t VCNL4035_Application::calcZXData()
{
    double left = sensor_data[0];
    double right = sensor_data[2];

    uint8_t check = 0;
    check = (left > ZX_min_threshold) && (right > ZX_min_threshold);

    if (check)
    {

        double left_dis = 0;
        double right_dis = 0;

        double A_1 = 1200;
        double B_1 = 60;
        double k_1 = -0.6015;
        double A_2 = 50;
        double k_2 = 1;
        double x = 0;

        double A_3 = 6.5;
        double B_3 = -0.0001;
        double k_3 = 2;
        double z = 0;

        /* Range To Distance */
        left_dis = (left + B_1);
        left_dis = A_1 * pow(left_dis, k_1);
        right_dis = (right + B_1);
        right_dis = A_1 * pow(right_dis, k_1);

        /* X Cordinate (Circle Intersection) */
        x = pow(left_dis, k_2) - pow(right_dis, k_2);
        x = A_2 * x;
        X_value = (int16_t)x;

        /* Z Cordinate */
        z = B_3 * pow(fabs(x), k_3);
        z += (left_dis + right_dis);
        z *= A_3;
        Z_value = (int16_t)z;
    }
    return check;
}

void VCNL4035_Application::subtractGestureNoise(uint16_t *data)
{

    if (gesture_noise_LDR1 >= data[0])
        data[0] = 0;
    else
        data[0] -= gesture_noise_LDR1;
    if (gesture_noise_LDR2 >= data[1])
        data[1] = 0;
    else
        data[1] -= gesture_noise_LDR2;
    if (gesture_noise_LDR3 >= data[2])
        data[2] = 0;
    else
        data[2] -= gesture_noise_LDR3;
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

bool VCNL4035_Application::isZXDataReady()
{
    if (gesture_flags.zx_data_ready)
    {
        gesture_flags.zx_data_ready = 0;
        return (1);
    }
    return (0);
}

/* Gets */
void VCNL4035_Application::getSensorData(uint16_t *data)
{
    data[0] = sensor_data[0];
    data[1] = sensor_data[1];
    data[2] = sensor_data[2];
}

void VCNL4035_Application::getZXData(int16_t *Z, int16_t *X)
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
