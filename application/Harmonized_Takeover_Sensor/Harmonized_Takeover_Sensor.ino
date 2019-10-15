/**
 * @file    Harmonized_Takeover_Sensor.ino
 * @brief   Application code for sensor module used in the harmonized takeover
 *          prototype.
 * @author  Luke Young (L S Young Electrical)
 *
 * The program reads proximity data from the VCNL4035 sensor board by Oliver 
 * Djurasovic useing the VCNL4035_App libray. It then transmits the data over 
 * serial to a host computer running the "Harmonized_Takeover_VMIDI.py" Python 3
 * script. The data received by the computer will depend on the current mode 
 * setting.
 * 
 * -----------------------------------------------------------------------------
 * Available modes are:
 * 
 * MIDI : This is standard operational mode, data from this mode is used to
 * control sound paramiters in a DAW.
 * 
 * RAW_DATA : Sends raw counts data from VCNL to computer. Use for debuging.
 * 
 * ZX_DATA : Sends non scaled ZX data to computer. Use for debuging.
 * 
 * ORB : Sends non scaled ZX data to computer only when an object is inside the 
 * orb boundary. Use for debuging.
 * -----------------------------------------------------------------------------
 * 
 * The "orb_timer" is the minimum time that must elapse between a exiting orb
 * and entering orb command is sent.
 * 
 * The application uses EEPROM to store the mode state when the device is 
 * switched off. When switched back on the device will boot into the last mode 
 * it was set to.  
 */

#define VCNL_INTERRUPT_PIN 2
#include <VCNL4035_App.h>

#define EEPROM_MODE_ADDR 0
#include <EEPROM.h>

#include <Handy_Arduino_Lib.h> // Used for timer

/* Modes */
enum modes
{
    MIDI = 0,
    RAW_DATA = 1,
    ZX_DATA = 2,
    ORB = 3,
    SWIPE = 4,
} mode;

/* VCNL4035 object */
VCNL4035_Application vcnl;

int16_t x_pos = 0;
int16_t z_pos = 0;

/* Orb timer */
uint8_t orb_status = 0;
uint8_t orb_timeout = 0;
Timer_ms orb_timer(200);

void setup()
{
    Serial.begin(115200);
    Serial.println("______Start Up______");

    orb_timer.reset_timer();

    /* VCNL Setup */
    vcnl.setIntPin(VCNL_INTERRUPT_PIN);
    vcnl.init(GESTURE);
    vcnl.setPsIntegrationTime(PS_IT_400us);
    vcnl.setLedCurrent(LED_I_100_mA);
    vcnl.calcGestureNoise(100);
    vcnl.setGestureNoiseSubtract(true);
    vcnl.gesture_flags.zx_enable = 1;
    vcnl.gesture_flags.zx_smooth_enable = 0;
    vcnl.gesture_flags.orb_detect_enable = 1;

    /* EEPROM Setup */
    EEPROM.get(EEPROM_MODE_ADDR, mode);
    Serial.print("Current Mode: ");
    Serial.println(mode);
}

void loop()
{
    vcnl.MainLoop();

    commincations_poll();

    switch (mode)
    {
    case MIDI:
        midi_mode();
        break;

    case RAW_DATA:
        raw_data_mode();
        break;

    case ZX_DATA:
        zx_data_mode();
        break;

    case ORB:
        orb_check();
        break;

    case SWIPE:
        // Not implemented yet
        break;
    }
}

void midi_mode()
{
    if (vcnl.gesture_flags.orb_status)
    {
        if (orb_timer.check_overflow())
        {
            orb_timeout = 1;
        }

        if (!orb_status && orb_timeout)
        {
            // Send Entering Orb Command
            Serial.println("A");
            orb_status = 1;
        }

        if (orb_status)
        {
            if (vcnl.isOrbDataReady())
            {
                vcnl.getZXData(&z_pos, &x_pos);
                // Send Orb Coordinates
                Serial.print("Z");
                Serial.println(z_pos);
                Serial.print("X");
                Serial.println(x_pos);
            }
        }
    }
    else
    {
        if (orb_status)
        {
            orb_timeout = 0;
            orb_timer.reset_timer();
            orb_status = 0;
            // Send Exiting Orb Command
            Serial.println("B");
        }
    }
}

void raw_data_mode()
{
    uint16_t sens_data[3];

    if (vcnl.isDataReady())
    {
        vcnl.getSensorData(sens_data);
        Serial.print(sens_data[0]);
        Serial.print(" , ");
        Serial.print(sens_data[1]);
        Serial.print(" , ");
        Serial.println(sens_data[2]);
    }
}

void zx_data_mode()
{
    int16_t x = 0;
    int16_t z = 0;

    if (vcnl.isZXDataReady())
    {
        vcnl.getZXData(&z, &x);
        Serial.print(z);
        Serial.print(", ");
        Serial.println(x);
    }
}

void orb_check()
{
    if (vcnl.isOrbDataReady())
    {
        vcnl.getZXData(&z_pos, &x_pos);
        Serial.print(z_pos);
        Serial.print(" , ");
        Serial.println(x_pos);
    }
}

void commincations_poll()
{
    if (Serial.available() > 0)
    {
        char until = '\n';
        uint8_t max_buffer_length = 20;
        char buffer[max_buffer_length + 1];
        uint8_t buffer_length = 0;

        buffer_length = Serial.readBytesUntil(until, buffer, 20);
        buffer[buffer_length] = '\0'; // Append Null char
        Serial.print("Input Command: ");
        Serial.println(buffer);

        if (String("MIDI") == buffer)
        {
            mode = MIDI;
            EEPROM.put(EEPROM_MODE_ADDR, mode);
        }
        else if (String("RAW_DATA") == buffer)
        {
            mode = RAW_DATA;
            EEPROM.put(EEPROM_MODE_ADDR, mode);
        }
        else if (String("ZX_DATA") == buffer)
        {
            mode = ZX_DATA;
            EEPROM.put(EEPROM_MODE_ADDR, mode);
        }
        else if (String("ORB") == buffer)
        {
            mode = ORB;
            EEPROM.put(EEPROM_MODE_ADDR, mode);
        }
        else if (String("SWIPE") == buffer)
        {
            mode = SWIPE;
            EEPROM.put(EEPROM_MODE_ADDR, mode);
        }
        else
        {
            Serial.println("Input Command: Not Valid");
        }
    }
}