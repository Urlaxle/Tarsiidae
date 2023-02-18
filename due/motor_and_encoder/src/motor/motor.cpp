#include <Arduino.h>

#include "motor.h"

static HardwareSerial* sr;

void motor_init(HardwareSerial* s) {
  sr = s;
  sr->begin(38400);
}


int motor_output(motor_mode_t mode, int out) {
  if (out > 63) {
    out = 63;
  } else if (out < -63) {
    out = -63;
  }

  out = out + 64;
  
  switch (mode) {
    case MOTOR_STOP:
      sr->write((byte)0);
      break;
    case MOTOR_1:
      sr->write(out);
      break;
    case MOTOR_2:
      sr->write(out+128);
      break;
    default:
      sr->write((byte)0);
      break;
  }

  return out;
}
