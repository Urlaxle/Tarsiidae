#include <Arduino.h>

#include "encoder.h"

int encoder_init(struct encoder *enc) {
  if (enc == NULL || enc->steps_per_rotation == 0) {
    return -1;
  }

  enc->steps = 0;
  enc->rotations = 0.0;

  /* Encoder GND */
  digitalWrite(enc->pin_gnd, LOW);
  pinMode(enc->pin_gnd, OUTPUT);

  /* Encoder 3.3V power supply */
  digitalWrite(enc->pin_3v3, HIGH);
  pinMode(enc->pin_3v3, OUTPUT);
  
  /* Encoder S0 and S1 signals */
  pinMode(enc->pin_s0, INPUT);
  pinMode(enc->pin_s1, INPUT);
  enc->s0_prev = digitalRead(enc->pin_s0);

  return 0;
}

int encoder_process(struct encoder *enc) {
  int s0, s1;
  
  if (enc == NULL) {
    return -1;
  }

  s0 = digitalRead(enc->pin_s0);
  s1 = digitalRead(enc->pin_s1);

  /* Look for rising edges of s0.
   * If s1 is high on a rising edge of s0, increase 'steps' by one.
   * If s1 is low on a rising edge of s0, decrease 'steps' by one.
   */
  if (s0 == HIGH && enc->s0_prev == LOW) {
    enc->steps += (s1 == HIGH) ? 1 : -1;
  }
  enc->s0_prev = s0;

  return 0;
}

int encoder_value_get(struct encoder *enc) {
  if (enc == NULL) {
    return 0;
  }

  return enc->steps;
}

float encoder_rotations_get(struct encoder *enc) {
  if (enc == NULL || enc->steps_per_rotation == 0) {
    return 0;
  }

  return (float)((float)enc->steps / (float)enc->steps_per_rotation);
}

int encoder_value_reset(struct encoder *enc) {
  if (enc == NULL) {
    return -1;
  }

  enc->steps = 0;
  return 0;
}
