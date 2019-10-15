/**
 * @file    VCNL4035_App.h
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

#ifndef VCNL4035_App_H
#define VCNL4035_App_H

#include <Arduino.h>
#include "VCNL4035_Lib.h"

/* Gesture Flags Structure */
typedef struct gesture_flags_s
{
    uint8_t noise_canc_enable : 1;
    uint8_t zx_enable : 1;
    uint8_t zx_smooth_enable : 1;
    uint8_t orb_detect_enable : 1;
    uint8_t gesture_recognition_enable : 1;
    uint8_t data_ready : 1;
    uint8_t zx_data_ready : 1;
    uint8_t orb_data_ready : 1;
    uint8_t gesture_recognition_data_ready : 1;
    uint8_t orb_status : 1;
    uint8_t activity_status : 1;
    uint8_t orb_data_cnt;
    uint8_t orb_data_cnt_limit;
} gesture_flags_t;

/* ZX Calculation Coeffecents */
typedef struct zx_coeff_s
{
    double A_1;
    double B_1;
    double k_1;
    double A_2;
    double k_2;
    double A_3;
    double B_3;
    double k_3;
} zx_coeff_t;

/* Orb Coordinates */
typedef struct orb_cord_s
{
    int16_t center_z_max;
    int16_t center_z_min;
    int16_t center_x_max;
    int16_t center_x_min;
    int16_t side_z_max;
    int16_t side_z_min;
    int16_t side_x_max;
    int16_t side_x_min;
    uint16_t center_IRED_max;
} orb_cord_t;

/* VCNL4035 Application Modes */
typedef enum VCNL4035_APP_MODES
{
    ACTIVE = 0,
    SLEEP = 1,
} VCNL4035_APP_MODES;

/* VCNL4035_Application Class (Parent Class: VCNL4035) */
class VCNL4035_Application : public VCNL4035
{
public:
    /* Constructor */
    VCNL4035_Application();

    /* Operational */
    void MainLoop();
    void ActiveLoop();
    void SleepLoop();
    void calcGestureNoise(uint8_t number_of_readings);
    uint8_t calcZXData(uint16_t left_data, uint16_t right_data);
    uint8_t calcOrb(int16_t Z, int16_t X);
    uint8_t checkOrb(orb_cord_t *limits, int16_t Z, int16_t X);
    void calcMovingMean(uint16_t left, uint16_t right);
    uint8_t calcGestureRecognition();
    void subtractGestureNoise(uint16_t *data);

    /* Checks */
    bool isDataReady();
    bool isGestureActivity();
    bool isZXDataReady();
    bool isOrbDataReady();
    bool isGestureRecognitionDataReady();

    /* Gets */
    void getSensorData(uint16_t *val);
    void getZXData(int16_t *Z, int16_t *X);
    void getGestureRecognitionData(int16_t *delay_samples);

    /* Settings */
    void setGestureNoiseSubtract(uint8_t state);
    void setGestureActiveThreshold(uint16_t threshold);
    void setSleepMode();
    void setActiveMode();
    void setZXEnable(uint8_t state);
    void setGestureRecognition(uint8_t state);

    /* Noise Varribles */
    uint16_t gesture_noise_LDR1;
    uint16_t gesture_noise_LDR2;
    uint16_t gesture_noise_LDR3;
    /* Parameter Varribles */
    gesture_flags_t gesture_flags;
    VCNL4035_APP_MODES current_mode;
    VCNL4035_APP_MODES prev_mode;
    uint16_t gesture_activity_threshold;
    /* Raw VCNL Data */
    uint16_t sensor_data[3];
    uint16_t mean_left_value;
    uint16_t mean_right_value;
    /* ZX Related */
    int16_t Z_value;
    int16_t X_value;
    zx_coeff_t ZX_coeff;
    uint8_t ZX_min_threshold;
    /* Gesture Recognition Related (Used In VCNL_4035_App_Swipes.cpp) */
    int16_t swipe_speed;
    int sample_skip = 3;
    uint16_t std_trigger = 25;
    uint16_t gesture_timeout_limit = 1000; //Not Implemented Yet
    /* Orb Related */
    orb_cord_t orb_cord;
    orb_cord_t orb_cord_leaving;
};

#endif /* VCNL4035_App_H */