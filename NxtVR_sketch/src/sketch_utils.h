#ifndef SKETCH_UTILS_H
#define SKETCH_UTILS_H
//#include <Wire.h>

static bool led_state;

static unsigned long nextt = millis();
static unsigned long current;

static void flip_led()
{
  current = millis();
  if (current > nextt)
  {
    nextt = current + 500;
    if (led_state)
    {
      //turns off
      led_state = false;
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      led_state = true;
      digitalWrite(LED_BUILTIN, LOW);
    }
  };
};

#endif
