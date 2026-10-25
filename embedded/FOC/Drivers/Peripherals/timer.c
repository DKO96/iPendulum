#include "timer.h"

#include <stm32f446xx.h>

void timer_pwm_init(void) {
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;

  TIM1->PSC = 0;
  TIM1->ARR = PWM_ARR;

  // Configure TIM1_CH1
  TIM1->CCMR1 &= ~TIM_CCMR1_OC1M;
  TIM1->CCMR1 |= (6U << TIM_CCMR1_OC1M_Pos);  // set PWM mode 1
  TIM1->CCMR1 |= TIM_CCMR1_OC1PE;             // enable preload
  TIM1->CCER &= ~TIM_CCER_CC1P;
  TIM1->CCER |= TIM_CCER_CC1E;

  // Configure TIM1_CH2
  TIM1->CCMR1 &= ~TIM_CCMR1_OC2M;
  TIM1->CCMR1 |= (6U << TIM_CCMR1_OC2M_Pos);  // set PWM mode 1
  TIM1->CCMR1 |= TIM_CCMR1_OC2PE;             // enable preload
  TIM1->CCER &= ~TIM_CCER_CC2P;
  TIM1->CCER |= TIM_CCER_CC2E;

  // Configure TIM1_CH3
  TIM1->CCMR2 &= ~TIM_CCMR2_OC3M;
  TIM1->CCMR2 |= (6U << TIM_CCMR2_OC3M_Pos);  // set PWM mode 1
  TIM1->CCMR2 |= TIM_CCMR2_OC3PE;             // enable preload
  TIM1->CCER &= ~TIM_CCER_CC3P;
  TIM1->CCER |= TIM_CCER_CC3E;

  // Center-aligned
  TIM1->CR1 &= ~TIM_CR1_CMS;
  TIM1->CR1 |= (3U << TIM_CR1_CMS_Pos);

  // Autoreload
  TIM1->CR1 |= TIM_CR1_ARPE;

  // Update generation
  TIM1->EGR |= TIM_EGR_UG;

  // Enable main output
  TIM1->BDTR |= TIM_BDTR_MOE;

  // Enable timer
  TIM1->CR1 |= TIM_CR1_CEN;
}
