#include "main.h"

#include <math.h>

#include "as5600.h"
#include "gpio.h"
#include "i2c.h"
#include "timer.h"
#include "uart.h"

#define CPU_HZ 180000000
#define TWO_PI (2.0f * (float)M_PI)
#define CPR 4096.0f
#define POLES 11.0f
#define V_BUS 12.0f
#define R_PHASE 5.55f
#define L_PHASE 0.0f
#define KV_RPM_PER_V 27.0f
#define VEL_MIN_DT 0.001f
#define VEL_TF 0.005f

void wrap_0_2pi(float *angle);
void clamp_duty(float *duty);
void set_phase_voltage(float v_d, float v_q, float theta_e);
void motor_offset(float *offset);
float flux_linkage(void);
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

  printS("Calibrate FOC \r\n");

  /* Motor offset */
  float offset = 0.0f;
  motor_offset(&offset);
  printF(offset);
  printS(" rad, ");
  printI((int32_t)lroundf(offset * CPR / TWO_PI));
  printS(" counts\r\n");

  /* Flux linkage */
  float lambda = flux_linkage();
  float kt = 1.5f * POLES * lambda;
  printS("lambda = ");
  printF(lambda);
  printS(" Wb, Kt = ");
  printF(kt);
  printS(" Nm/A\r\n");

  /* Estimate velocity */
  float theta_prev = as_angle(I2C1) * TWO_PI / CPR;
  uint32_t t_prev = DWT->CYCCNT;
  float omega_m = 0.0f;

  float torque_ref = 0.6f;
  float i_q_ref = torque_ref / kt;
  printS("i_q = ");
  printF(i_q_ref);
  printS(" A\r\n");

  printS("Start FOC \r\n");
  while (1) {
    float theta_m = as_angle(I2C1) * TWO_PI / CPR;

    float theta_e = POLES * (theta_m - offset);
    wrap_0_2pi(&theta_e);

    uint32_t t_curr = DWT->CYCCNT;
    float dt = (float)(t_curr - t_prev) / CPU_HZ;

    if (dt >= VEL_MIN_DT) {
      float d_theta = theta_m - theta_prev;

      if (d_theta > (float)M_PI) {
        d_theta -= TWO_PI;
      } else if (d_theta < -(float)M_PI) {
        d_theta += TWO_PI;
      }

      float omega_raw = d_theta / dt;
      float alpha = dt / (VEL_TF + dt);
      omega_m += alpha * (omega_raw - omega_m);

      theta_prev = theta_m;
      t_prev = t_curr;
    }

    float omega_e = POLES * omega_m;

    float v_q = R_PHASE * i_q_ref + lambda * omega_e;
    float v_d = -omega_e * L_PHASE * i_q_ref;

    set_phase_voltage(v_d, v_q, theta_e);
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

void set_phase_voltage(float v_d, float v_q, float theta_e) {
  float v_lim = V_BUS / sqrtf(3.0f);
  float mag = sqrtf(v_d * v_d + v_q * v_q);
  if (mag > v_lim) {
    float k = v_lim / mag;
    v_d *= k;
    v_q *= k;
  }

  float v_alph = v_d * cosf(theta_e) - v_q * sinf(theta_e);
  float v_beta = v_d * sinf(theta_e) + v_q * cosf(theta_e);

  float v_a = v_alph;
  float v_b = (-v_alph + sqrtf(3.0f) * v_beta) / 2.0f;
  float v_c = (-v_alph - sqrtf(3.0f) * v_beta) / 2.0f;

  float v_max = fmaxf(v_a, fmaxf(v_b, v_c));
  float v_min = fminf(v_a, fminf(v_b, v_c));
  float v_off = -(v_max + v_min) / 2.0f;

  float v_a_out = v_a + v_off;
  float v_b_out = v_b + v_off;
  float v_c_out = v_c + v_off;

  float duty_a = 0.5f + (v_a_out / V_BUS);
  float duty_b = 0.5f + (v_b_out / V_BUS);
  float duty_c = 0.5f + (v_c_out / V_BUS);

  clamp_duty(&duty_a);
  clamp_duty(&duty_b);
  clamp_duty(&duty_c);

  TIM1->CCR1 = (uint32_t)(duty_a * PWM_ARR);
  TIM1->CCR2 = (uint32_t)(duty_b * PWM_ARR);
  TIM1->CCR3 = (uint32_t)(duty_c * PWM_ARR);
}

void motor_offset(float *offset) {
  float v_q = 0.0f;
  float v_d = 2.0f;
  float theta_e = 0.0f;
  float v_bus = 12.0f;

  set_phase_voltage(v_d, v_q, theta_e);

  delay_ms(1000);

  float s = 0.0f;
  float c = 0.0f;
  uint16_t samples = 20;

  for (uint16_t i = 0; i < samples; i++) {
    float angle = as_angle(I2C1) * TWO_PI / CPR;
    s += sinf(angle);
    c += cosf(angle);

    delay_ms(10);
  }

  v_q = 0.0f;
  v_d = 0.0f;
  theta_e = 0.0f;

  set_phase_voltage(v_d, v_q, theta_e);

  float avg_encoder = atan2f(s, c);
  wrap_0_2pi(&avg_encoder);

  *offset = avg_encoder;
}

float flux_linkage(void) {
  float kv_rad = KV_RPM_PER_V * (TWO_PI / 60.0f);
  return 1.0f / (sqrtf(3.0f) * kv_rad * POLES);
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