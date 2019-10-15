#include <FastLED.h>

// #define DEBUG_LED_SERIAL_COMS

// Hardwere Defintions
#define LED_PIN 3
#define COLOR_ORDER BRG
#define CHIPSET WS2811
#define NUM_LEDS 5

// __________GLOBAL VARRIBLES__________
// Addressable LED Array
CRGB leds[NUM_LEDS];

// Loop delay time
uint16_t delay_time_ms = 1;

// LED Effects
uint8_t total_brightness = 255;

uint8_t hue = 0;
uint8_t hue_inital = 0;

uint8_t bright_value = 0;
uint8_t bright_value_delta = 3;

uint8_t black_fade_value = 5;

uint8_t fade_in_flag = 0;

uint8_t BPM = 60;

// Incoming Data (Comminacations)
uint8_t incomingByte = 0;
uint8_t lastByte = 0;
uint8_t i = 0;

int16_t x_inital = -1;
uint8_t x_current = 0;
int8_t x_delta = 0;
uint8_t x_scale = 2;

int16_t z_inital = -1;
uint8_t z_current = 0;
int8_t z_delta = 0;
uint8_t z_scale = 2;

// State Machine
enum orb_states
{
  HAND_OUT = 0,
  HAND_ENTERING = 1,
  HAND_IN = 2,
  HAND_LEAVING = 3,
} orb_state = HAND_OUT;

void setup()
{
  Serial.begin(57600);
  Serial.println("resetting");

  LEDS.addLeds<CHIPSET, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  LEDS.setBrightness(total_brightness);

  /* Turn all LEDS Off */
  for (i = 0; i < NUM_LEDS; i++)
  {
    leds[i] = 0;
  }
  LEDS.show();
}

void loop()
{
  // Check for external serial comminacations
  communications_poll();

  if (orb_state == HAND_ENTERING)
  {
    // Select Random Color
    hue_inital = random8();
    // Reset Brightness and Fade In
    bright_value = 0;
    fade_in_flag = 1;
    // Update Orb State
    orb_state = HAND_IN;
  }

  else if (orb_state == HAND_IN)
  {
    // Update Color Based On Hands X Value
    hue = hue_inital + x_delta;

    // Fade In LED
    if (fade_in_flag == 1)
    {
      // Saturation addition (Dose not overflow)
      bright_value = qadd8(bright_value, bright_value_delta);
      if (bright_value == 255 && beatsin8(BPM, 2 * z_current, 255) == 255)
        fade_in_flag = 0; // Fade In Complete
    }
    // Oscillate LED
    else
    {
      bright_value = beatsin8(BPM, 2 * z_current, 255);
    }

    // Update LED Brightness
    for (i = 0; i < NUM_LEDS; i++)
    {
      // Note: brighten8_raw(bright_value):
      // Scales a linear increase in brightness to a visually linear value
      leds[i].setHSV(hue, 255, brighten8_raw(bright_value));
    }
  }

  else if (orb_state == HAND_LEAVING)
  {
    // Fade Out LEDS
    for (i = 0; i < NUM_LEDS; i++)
    {
      leds[i].fadeToBlackBy(black_fade_value);
    }
    // Reset Z and Y Values
    x_delta = 0;
    z_delta = 0;
    x_inital = -1;
    z_inital = -1;
  }

  else if (orb_state == HAND_OUT)
  {
    // Dosent Do Anything Yet
  }

  delay(delay_time_ms);
  LEDS.show();
}

void communications_poll(void)
{
  /* Read From Serial */
  if (Serial.available() > 0)
  {
    lastByte = incomingByte;
    incomingByte = Serial.read();

#ifdef DEBUG_LED_SERIAL_COMS
    Serial.println((char)incomingByte);
#endif

    if (incomingByte == 'A') // Hand Enters Orb
      orb_state = HAND_ENTERING;

    else if (incomingByte == 'B') // Hand Exits Orb
      orb_state = HAND_LEAVING;

    else if (incomingByte == 'X')
    {
      int16_t x_temp;
      x_temp = Serial.parseInt();

      if ((x_temp >= 0) && (x_temp <= 255)) // Valid Range
      {
        if (x_inital == -1)
        {
          x_inital = x_temp;
#ifdef DEBUG_LED_SERIAL_COMS
          Serial.print("x_inital: ");
          Serial.print(x_inital);
#endif
        }
        else
        {
          x_current = x_temp;
          x_delta = (x_temp - x_inital) / x_scale;
#ifdef DEBUG_LED_SERIAL_COMS
          Serial.print("x_delta: ");
          Serial.print(x_delta);
#endif
        }
      }
    }

    else if (incomingByte == 'Z')
    {
      int16_t z_temp;
      z_temp = Serial.parseInt();

      if ((z_temp >= 0) && (z_temp <= 255)) // Valid Range
      {
        if (z_inital == -1)
        {
          z_inital = z_temp;
#ifdef DEBUG_LED_SERIAL_COMS
          Serial.print("z_inital: ");
          Serial.print(z_inital);
#endif
        }
        else
        {
          z_current = z_temp;
          z_delta = (z_temp - z_inital) / z_scale;
#ifdef DEBUG_LED_SERIAL_COMS
          Serial.print("z_delta: ");
          Serial.print(z_delta);
#endif
        }
      }
    }

    else if (incomingByte == 'C')
    {
      int16_t bpm_temp = Serial.parseInt();
      if ((bpm_temp >= 0) && (bpm_temp <= 255)) // Valid Range
        BPM = bpm_temp;
    }

    else if (incomingByte == 'D')
    {
      int16_t bright_temp = Serial.parseInt();
      if ((bright_temp >= 0) && (bright_temp <= 255)) // Valid Range
      {
        total_brightness = bright_temp;
        LEDS.setBrightness(total_brightness);
      }
    }
  }
}
