#include "main.h"

#include <math.h>

#include "as5600.h"
#include "gpio.h"
#include "i2c.h"
#include "timer.h"
#include "uart.h"

#define CPR 4096.0f
#define POLES 11.0f
#define OFFSET 0.0f
#define PRINT_EVERY 8

void wrap_0_2pi(float *angle);
void clamp_duty(float *duty);
static void print_field(int32_t value);
static void print_debug(uint32_t ticks, float v_a_out, float v_b_out,
                        float v_c_out, float duty_a, float duty_b,
                        float duty_c);

int main() {
  /* Initialize hardware */
  system_init();
  gpio_init();
  uart_init();
  timer_pwm_init();
  i2c_init(I2C1);
  as_init(I2C1);

  float v_d = 0.0f;
  float v_q = 7.0f;

  float theta_m = 0.0f;
  float theta_e = 0.0f;

  float v_alph = 0.0f;
  float v_beta = 0.0f;

  float v_a = 0.0f;
  float v_b = 0.0f;
  float v_c = 0.0f;

  float v_min = 0.0f;
  float v_max = 0.0f;
  float v_off = 0.0f;

  float v_a_out = 0.0f;
  float v_b_out = 0.0f;
  float v_c_out = 0.0f;
  float v_bus = 12.0f;

  float duty_a = 0.0f;
  float duty_b = 0.0f;
  float duty_c = 0.0f;

  printS("Start FOC \r\n");

  int16_t encoder;

  while (1) {
    encoder = as_angle(I2C1);

    theta_m = encoder * 2.0f * (float)M_PI / CPR;
    theta_e = POLES * (theta_m - OFFSET);
    wrap_0_2pi(&theta_e);

    v_alph = v_d * cosf(theta_e) - v_q * sinf(theta_e);
    v_beta = v_d * sinf(theta_e) + v_q * cosf(theta_e);

    v_a = v_alph;
    v_b = (-v_alph + sqrtf(3.0f) * v_beta) / 2.0f;
    v_c = (-v_alph - sqrtf(3.0f) * v_beta) / 2.0f;

    v_max = fmaxf(v_a, fmaxf(v_b, v_c));
    v_min = fminf(v_a, fminf(v_b, v_c));
    v_off = -(v_max + v_min) / 2.0f;

    v_a_out = v_a + v_off;
    v_b_out = v_b + v_off;
    v_c_out = v_c + v_off;

    duty_a = 0.5f + (v_a_out / v_bus);
    duty_b = 0.5f + (v_b_out / v_bus);
    duty_c = 0.5f + (v_c_out / v_bus);

    clamp_duty(&duty_a);
    clamp_duty(&duty_b);
    clamp_duty(&duty_c);

    TIM1->CCR1 = (uint32_t)(duty_a * PWM_ARR);
    TIM1->CCR2 = (uint32_t)(duty_b * PWM_ARR);
    TIM1->CCR3 = (uint32_t)(duty_c * PWM_ARR);
  }

  return 0;
}

void wrap_0_2pi(float *angle) {
  float x = fmodf(*angle, 2.0f * (float)M_PI);
  if (x < 0.0f) {
    x += 2.0f * M_PI;
  }

  *angle = x;
}

void clamp_duty(float *duty) {
  if (*duty > 1.0f) {
    *duty = 1.0f;
  } else if (*duty < 0.0f) {
    *duty = 0.0f;
  }
}

static void print_field(int32_t value) {
  printI(value);
  printS(",");
}

static void print_debug(uint32_t ticks, float v_a_out, float v_b_out,
                        float v_c_out, float duty_a, float duty_b,
                        float duty_c) {
  print_field((int32_t)ticks);

  // phase voltages in millivolts
  print_field((int32_t)lroundf(v_a_out * 1000.0f));
  print_field((int32_t)lroundf(v_b_out * 1000.0f));
  print_field((int32_t)lroundf(v_c_out * 1000.0f));

  // duty in tenths of a percent (789 = 78.9%)
  print_field((int32_t)lroundf(duty_a * 1000.0f));
  print_field((int32_t)lroundf(duty_b * 1000.0f));
  print_field((int32_t)lroundf(duty_c * 1000.0f));

  // read CCRs back from the registers, so this shows what the timer actually
  // got
  print_field((int32_t)TIM1->CCR1);
  print_field((int32_t)TIM1->CCR2);
  printI((int32_t)TIM1->CCR3);  // last field: no trailing comma

  printS("\r\n");
}