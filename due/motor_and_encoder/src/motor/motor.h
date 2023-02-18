#ifndef MOTOR_H__
#define MOTOR_H__

#include <Arduino.h>

typedef enum {
  MOTOR_STOP,
  MOTOR_1,
  MOTOR_2
} motor_mode_t;

void motor_init(HardwareSerial *s);

int motor_output(motor_mode_t mode, int out);

#endif /* MOTOR_H__ */
