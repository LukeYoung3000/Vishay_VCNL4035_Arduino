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

    gesture_flags.orb_status = 0;
    gesture_flags.orb_data_cnt = 0;
    gesture_flags.orb_data_cnt_limit = 4;

    ZX_coeff.A_1 = 1200;
    ZX_coeff.B_1 = 60;
    ZX_coeff.k_1 = -0.6015;
    ZX_coeff.A_2 = 50;
    ZX_coeff.k_2 = 1;
    ZX_coeff.A_3 = 6.5;
    ZX_coeff.B_3 = -0.0001;
    ZX_coeff.k_3 = 2;

    orb_cord.center_z_max = 850;
    orb_cord.center_z_min = 200;
    orb_cord.center_x_max = 80;
    orb_cord.center_x_min = -80;
    orb_cord.side_z_max = 750;
    orb_cord.side_z_min = 170;
    orb_cord.side_x_max = 440;
    orb_cord.side_x_min = -440;
    orb_cord.center_IRED_max = 2500;

    orb_cord_leaving.center_z_max = 870;
    orb_cord_leaving.center_z_min = 180;
    orb_cord_leaving.center_x_max = 95;
    orb_cord_leaving.center_x_min = -95;
    orb_cord_leaving.side_z_max = 770;
    orb_cord_leaving.side_z_min = 160;
    orb_cord_leaving.side_x_max = 480;
    orb_cord_leaving.side_x_min = -480;
    orb_cord_leaving.center_IRED_max = 2700;
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
            uint16_t left_data = sensor_data[0];
            uint16_t right_data = sensor_data[2];
            if (gesture_flags.zx_smooth_enable)
            {
                // Smooth the raw data
                calcMovingMean(left_data, right_data);
                left_data = mean_left_value;
                right_data = mean_right_value;
            }

            if (calcZXData(left_data, right_data))
            {
                gesture_flags.zx_data_ready = 1;

                // Orb Detection
                if (gesture_flags.orb_detect_enable)
                {
                    if (calcOrb(Z_value, X_value))
                    {
                        gesture_flags.orb_data_ready = 1;

                        if (gesture_flags.orb_data_cnt < gesture_flags.orb_data_cnt_limit)
                        {
                            gesture_flags.orb_data_cnt++;
                        }
                        else
                        {
                            gesture_flags.orb_status = 1;
                        }
                    }
                    else
                    {
                        if (gesture_flags.orb_data_cnt > 0)
                        {
                            gesture_flags.orb_data_cnt--;
                        }
                        else
                        {
                            gesture_flags.orb_status = 0;
                        }
                    }
                }
            }
            else
            {
                // Set orb status low if we go out of range
                gesture_flags.orb_data_cnt = 0;
                gesture_flags.orb_status = 0;
            }
        }

        // Detect Valid Gestures
        if (gesture_flags.gesture_recognition_enable)
        {
            if (calcGestureRecognition())
                gesture_flags.gesture_recognition_data_ready = 1;
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

uint8_t VCNL4035_Application::calcZXData(uint16_t left_data, uint16_t right_data)
{
    double left = left_data;
    double right = right_data;

    uint8_t check = 0;
    check = (left > ZX_min_threshold) && (right > ZX_min_threshold);

    if (check)
    {
        double left_dis = 0;
        double right_dis = 0;

        double x = 0;
        double z = 0;

        /* Range To Distance */
        left_dis = (left + ZX_coeff.B_1);
        left_dis = ZX_coeff.A_1 * pow(left_dis, ZX_coeff.k_1);
        right_dis = (right + ZX_coeff.B_1);
        right_dis = ZX_coeff.A_1 * pow(right_dis, ZX_coeff.k_1);

        /* X Cordinate (Circle Intersection) */
        x = pow(left_dis, ZX_coeff.k_2) - pow(right_dis, ZX_coeff.k_2);
        x = ZX_coeff.A_2 * x;
        X_value = (int16_t)x;

        /* Z Cordinate */
        z = ZX_coeff.B_3 * pow(fabs(x), ZX_coeff.k_3);
        z += (left_dis + right_dis);
        z *= ZX_coeff.A_3;
        Z_value = (int16_t)z;
    }
    return check;
}

uint8_t VCNL4035_Application::calcOrb(int16_t Z, int16_t X)
{
    // Calculate if we are in the orb based on our current location
    if (gesture_flags.orb_status)
    {
        return checkOrb(&orb_cord_leaving, Z, X);
    }
    else
    {
        return checkOrb(&orb_cord, Z, X);
    }
}

uint8_t VCNL4035_Application::checkOrb(orb_cord_t *limits, int16_t Z, int16_t X)
{
    if (sensor_data[1] < limits->center_IRED_max)
    {
        uint8_t check = 0;
        check = ((X < limits->center_x_max) &&
                 (X > limits->center_x_min) &&
                 (Z < limits->center_z_max) &&
                 (Z > limits->center_z_min));

        if (check)
        {
            return 1;
        }
        else if ((X < limits->side_x_max) &&
                 (X > limits->side_x_min))
        {
            if ((Z < limits->side_z_max) && (Z > limits->side_z_min))
            {
                return 1;
            }
        }
    }
    return 0;
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

bool VCNL4035_Application::isOrbDataReady()
{
    if (gesture_flags.orb_data_ready)
    {
        gesture_flags.orb_data_ready = 0;
        return (1);
    }
    return (0);
}

bool VCNL4035_Application::isGestureRecognitionDataReady()
{
    if (gesture_flags.gesture_recognition_data_ready)
    {
        gesture_flags.gesture_recognition_data_ready = 0;
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

void VCNL4035_Application::getGestureRecognitionData(int16_t *delay_samples)
{
    *delay_samples = swipe_speed;
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

void VCNL4035_Application::setZXEnable(uint8_t state)
{
    if (state > 0)
        gesture_flags.zx_enable = 1;
    else
        gesture_flags.zx_enable = 0;
}

void VCNL4035_Application::setGestureRecognition(uint8_t state)
{
    if (state > 0)
        gesture_flags.gesture_recognition_enable = 1;
    else
        gesture_flags.gesture_recognition_enable = 0;
}
