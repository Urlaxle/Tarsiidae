#ifndef ENCODER2_H__
#define ENCODER2_H__

#include <Arduino.h>

/*
 * Double check and ensure correct pins are set
 * to avoid damage to Arduino or encoders.
 */
struct encoder {
  /* configurations */
  int pin_s0;
  int pin_s1;
  int pin_3v3;
  int pin_gnd;
  unsigned int steps_per_rotation;
  /* states */
  int s0_prev;
  int steps;
  float rotations;
};

int encoder_init(struct encoder *enc);

int encoder_process(struct encoder *enc);

float encoder_rotations_get(struct encoder *enc);

int encoder_value_reset(struct encoder *enc);

#endif /* ENCODER2_H__ */
