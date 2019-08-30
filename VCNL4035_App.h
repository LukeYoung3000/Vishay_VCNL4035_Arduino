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
    uint8_t gesture_recognition_enable : 1;
    uint8_t data_ready : 1;
    uint8_t zx_data_ready : 1;
    uint8_t activity_status : 1;
} gesture_flags_t;

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
    uint8_t calcZXData();
    void subtractGestureNoise(uint16_t *data);
    //uint16_t calcZX(uint16_t *raw_data, uint16_t *Z, uint16_t *X);

    /* Checks */
    bool isDataReady();
    bool isGestureActivity();
    bool isZXDataReady();

    /* Gets */
    void getSensorData(uint16_t *val);
    void getZXData(int16_t *Z, int16_t *X);
    //void getGestureRecognitionData();

    /* Settings */
    void setGestureNoiseSubtract(uint8_t state);
    void setGestureActiveThreshold(uint16_t threshold);
    void setSleepMode();
    void setActiveMode();

    uint16_t gesture_noise_LDR1;
    uint16_t gesture_noise_LDR2;
    uint16_t gesture_noise_LDR3;
    gesture_flags_t gesture_flags;
    VCNL4035_APP_MODES current_mode;
    VCNL4035_APP_MODES prev_mode;
    uint16_t gesture_activity_threshold;
    uint16_t sensor_data[3];
    int16_t Z_value;
    int16_t X_value;
    uint8_t ZX_min_threshold;
};

#endif /* VCNL4035_App_H */