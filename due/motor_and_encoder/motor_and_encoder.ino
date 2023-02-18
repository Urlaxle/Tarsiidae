#include <Arduino.h>
#include "src/motor/motor.h"
#include "src/encoder/encoder.h"

#define BLINK_LED_PIN 13
#define BLINK_LED_PERIOD_MS 500
#define ENCODERS_PRINT_PERIOD_MS 500
#define ENCODERS_SEND_PERIOD_MS 67

void blink_led_init(void) {
  pinMode(BLINK_LED_PIN, OUTPUT);
}

void blink_led_process(void) {
  static int toggle = LOW;
  static unsigned long t_end = 0;
  unsigned long t = millis();

  if (t < t_end) {
    return;
  }
  t_end = t + BLINK_LED_PERIOD_MS;

  digitalWrite(BLINK_LED_PIN, toggle);
  toggle = (toggle == LOW) ? HIGH : LOW;
}

static struct encoder encLF = {
  .pin_s0  = 50,
  .pin_s1  = 52,
  .pin_3v3 = 51,
  .pin_gnd = 53,
  .steps_per_rotation = 3000
};
static struct encoder encRF = {
  .pin_s0  = 48,
  .pin_s1  = 46,
  .pin_3v3 = 47,
  .pin_gnd = 49,
  .steps_per_rotation = 3000
};
static struct encoder encLB = {
  .pin_s0  = 42,
  .pin_s1  = 44,
  .pin_3v3 = 43,
  .pin_gnd = 45,
  .steps_per_rotation = 3000
};
static struct encoder encRB = {
  .pin_s0  = 40,
  .pin_s1  = 38,
  .pin_3v3 = 39,
  .pin_gnd = 41,
  .steps_per_rotation = 3000
};
static struct encoder *encs[] = { &encLF, &encRF, &encLB, &encRB };
static const char *encs_labels[] = { "LF", "RF", "LB", "RB" };
static const size_t encs_len = sizeof(encs)/sizeof(encs[0]);
static const char *encs_msg_op = "ENCS";
static const size_t encs_msg_op_len = sizeof("ENCS")-1;
static bool encs_send_msg = false;

void encoders_init(void) {
  int err;

  for (int i = 0; i < encs_len; i++) {
    err = encoder_init(encs[i]);
    if (!err) {
      continue;
    }
    Serial.print("Failed encoder ");
    Serial.print(encs_labels[i]);
    Serial.print(" initialization, err ");
    Serial.println(err);
  }
}

void encoders_process(void) {
  int err;

  for (int i = 0; i < encs_len; i++) {
    err = encoder_process(encs[i]);
    if (!err) {
      continue;
    }
    Serial.print("Failed encoder ");
    Serial.print(encs_labels[i]);
    Serial.print(" processing, err ");
    Serial.println(err);
    delay(1000);
  }
}

void encoders_send(void) {
  static unsigned long t_end = 0;
  unsigned long t = millis();
  const size_t msg_len = encs_msg_op_len + encs_len*sizeof(float) + sizeof('\n');
  byte buf[msg_len] = {0};
  byte *wp = &buf[0];
  float rotations;

  if (t < t_end) {
    return;
  }
  t_end = t + ENCODERS_SEND_PERIOD_MS;

  if (encs_send_msg == false) {
    return;
  }

  memcpy(wp, encs_msg_op, encs_msg_op_len);
  wp += encs_msg_op_len;
  for (int i = 0; i < sizeof(encs)/sizeof(encs[0]); i++) {
    rotations = encoder_rotations_get(encs[i]);
    memcpy(wp, (byte *)&rotations, sizeof(rotations));
    wp += sizeof(rotations);
  }
  *wp = '\n';
  
  SerialUSB.write(buf, sizeof(buf));
}

void encoders_print(void) {
  static unsigned long t_end = 0;
  unsigned long t = millis();
  float rotations;

  if (t < t_end) {
    return;
  }
  t_end = t + ENCODERS_PRINT_PERIOD_MS;

  if (encs_send_msg == false) {
    return;
  }

  for (int i = 0; i < encs_len; i++) {
    rotations = encoder_rotations_get(encs[i]);
    Serial.print(encs_labels[i]);
    Serial.print(": ");
    if (i < encs_len-1) {
      Serial.print(rotations, 3);
      Serial.print(", ");
    } else {
      Serial.println(rotations, 3);
    }
  }
}

static const char motor_stop_str[] = "MOTOR_ST";
static const char motor_1_str[] = "MOTOR_1";
static const char motor_2_str[] = "MOTOR_2";
static const char enc_send_str[] = "ENC_SEND";
static const char enc_stop_str[] = "ENC_STOP";
static size_t nm_motor_stop = 0;
static size_t nm_motor_1 = 0;
static size_t nm_motor_2 = 0;
static size_t nm_enc_send = 0;
static size_t nm_enc_stop = 0;
static bool motor_stop_cmd_received = false;
static bool motor_1_cmd_received = false;
static bool motor_2_cmd_received = false;
static bool enc_send_cmd_received = false;
static bool enc_stop_cmd_received = false;

void process_serial_cmds(Serial_ *s) {
//void process_serial_cmds(HardwareSerial *s) {
  char buf[32];
  size_t buf_idx = 0;
  size_t num = s->readBytes(&buf[0], sizeof(buf));
  if (num == 0) {
    return;
  }
  
  while (buf_idx < num) {
    if (motor_1_cmd_received) {
      Serial.write(&motor_1_str[0], sizeof(motor_1_str));
      Serial.print(": ");
      Serial.println((signed char)buf[buf_idx]);
      motor_output(MOTOR_1, (signed char)buf[buf_idx]);
      buf_idx += 1;
      motor_1_cmd_received = false;
    } else if (motor_2_cmd_received) {
      Serial.write(&motor_2_str[0], sizeof(motor_2_str));
      Serial.print(": ");
      Serial.println((signed char)buf[buf_idx]);
      motor_output(MOTOR_2, (signed char)buf[buf_idx]);
      buf_idx += 1;
      motor_2_cmd_received = false;
    }

    if (buf_idx >= num) {
      return;
    }
    
    if (buf[buf_idx] == motor_1_str[nm_motor_1]) {
      nm_motor_1 += 1;
    } else {
      nm_motor_1 = 0;
    }
    if (buf[buf_idx] == motor_2_str[nm_motor_2]) {
      nm_motor_2 += 1;
    } else {
      nm_motor_2 = 0;
    }
    if (buf[buf_idx] == motor_stop_str[nm_motor_stop]) {
      nm_motor_stop += 1;
    } else {
      nm_motor_stop = 0;
    }
    if (buf[buf_idx] == enc_send_str[nm_enc_send]) {
      nm_enc_send += 1;
    } else {
      nm_enc_send = 0;
    }
    if (buf[buf_idx] == enc_stop_str[nm_enc_stop]) {
      nm_enc_stop += 1;
    } else {
      nm_enc_stop = 0;
    }

    if (nm_motor_1 == sizeof(motor_1_str)-1) {
        motor_1_cmd_received = true;
        nm_motor_1 = 0;
        nm_motor_2 = 0;
        nm_motor_stop = 0;
        nm_enc_send = 0;
        nm_enc_stop = 0;
    } else if (nm_motor_2 == sizeof(motor_2_str)-1) {
        motor_2_cmd_received = true;
        nm_motor_1 = 0;
        nm_motor_2 = 0;
        nm_motor_stop = 0;
        nm_enc_send = 0;
        nm_enc_stop = 0;
    } else if (nm_motor_stop == sizeof(motor_stop_str)-1) {
        motor_stop_cmd_received = true;
        nm_motor_1 = 0;
        nm_motor_2 = 0;
        nm_motor_stop = 0;
        nm_enc_send = 0;
        nm_enc_stop = 0;
    } else if (nm_enc_send == sizeof(enc_send_str)-1) {
        enc_send_cmd_received = true;
        nm_motor_1 = 0;
        nm_motor_2 = 0;
        nm_motor_stop = 0;
        nm_enc_send = 0;
        nm_enc_stop = 0;
    } else if (nm_enc_stop == sizeof(enc_stop_str)-1) {
        enc_stop_cmd_received = true;
        nm_motor_1 = 0;
        nm_motor_2 = 0;
        nm_motor_stop = 0;
        nm_enc_send = 0;
        nm_enc_stop = 0;
    }

    if (motor_stop_cmd_received) {
      Serial.write(&motor_stop_str[0], sizeof(motor_stop_str));
      Serial.println("");
      motor_output(MOTOR_STOP, 0);
      motor_stop_cmd_received = false;
    } else if (enc_send_cmd_received) {
      Serial.write(&enc_send_str[0], sizeof(enc_send_str));
      Serial.println("");
      encs_send_msg = true;
      enc_send_cmd_received = false;
    } else if (enc_stop_cmd_received) {
      Serial.write(&enc_stop_str[0], sizeof(enc_stop_str));
      Serial.println("");
      encs_send_msg = false;
      enc_stop_cmd_received = false;
    }
    
    buf_idx += 1;
  }
}

void setup() {
  /* Initialize serial to receive motor commands over. */
  SerialUSB.begin(115200);

  /* Initialize serial for debugging and motor commands. Wait for communication to be established. */
  Serial.setTimeout(0);
  Serial.begin(115200);
  //while(!Serial);
  Serial.println("simple motor control sample started");

  /* Initialize debug led. */
  blink_led_init();

  /* Initialize encoders. */
  encoders_init();

  /* Initialize motor driver, using Serial1. */
  motor_init(&Serial1);
}

void loop() {
  blink_led_process();

  /* Get values from encoders. */
  encoders_process();

  /* Debug print values on Serial. */
  encoders_print();

  /* Send values over SerialUSB. */
  encoders_send();

  /* Read in and handle commands from SerialUSB/Serial. */
  process_serial_cmds(&SerialUSB);
  //process_serial_cmds(&Serial);
}
