#include "vehicle_io.h"
#include "current_ina219.h"
#include "stm32f7xx.h"
#include <stdio.h>
#include <string.h>

/* Owns TIM4, EXTI8/9/10/11 and PC10/11, PC8/9, PD14/15, PF12/13.
 * Motor 2 encoder uses both-edge interrupts on PC8=A, PC9=B.
 * TIM1 and the existing USART/IMU configuration are left to Simulink.
 * Called by the base-rate 1 ms task. No motor is enabled at initialization.
 */
static void alternate(GPIO_TypeDef *gpio, unsigned pin, unsigned af)
{
    unsigned shift = pin * 2U;
    gpio->MODER = (gpio->MODER & ~(3UL << shift)) | (2UL << shift);
    gpio->PUPDR &= ~(3UL << shift); /* No pull-up on 5 V encoder inputs. */
    gpio->OTYPER &= ~(1UL << pin);
    gpio->AFR[pin / 8U] = (gpio->AFR[pin / 8U] & ~(15UL << ((pin % 8U)*4U)))
                          | (af << ((pin % 8U)*4U));
}

/* PC8/PC9 are GPIO encoder inputs, not TIM8 alternate functions.
 * Interrupt decoding preserves edges between the 1 ms base-rate calls.
 * No pull resistors: retain external encoder signal conditioning.
 */
static volatile uint16_t encoder2_count;
static uint8_t encoder2_previous;
static uint8_t encoder2_seen;
static volatile uint16_t encoder2_a_edges, encoder2_b_edges, encoder2_invalid;
static const int8_t encoder2_step[16] = {
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
};
void EXTI9_5_IRQHandler(void)
{
    const uint32_t mask = (1UL << 8) | (1UL << 9);
    uint32_t pending = EXTI->PR & mask;
    if (pending) {
        uint8_t state;
        EXTI->PR = pending;
        state = (uint8_t)((GPIOC->IDR >> 8) & 3U);
        if (pending & (1UL << 8)) encoder2_a_edges++;
        if (pending & (1UL << 9)) encoder2_b_edges++;
        if ((state ^ encoder2_previous) == 3U) encoder2_invalid++;
        encoder2_count = (uint16_t)(encoder2_count +
            encoder2_step[(encoder2_previous << 2) | state]);
        encoder2_previous = state;
    }
}
static void encoder2_init(void)
{
    const uint32_t mask = (1UL << 8) | (1UL << 9);
    EXTI->IMR &= ~mask;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    GPIOC->MODER &= ~((3UL << 16) | (3UL << 18));
    GPIOC->PUPDR &= ~((3UL << 16) | (3UL << 18));
    /* EXTI8 and EXTI9 select port C (2). */
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~0xffUL) | 0x22UL;
    EXTI->RTSR |= mask;
    EXTI->FTSR |= mask;
    encoder2_count = 0;
    encoder2_previous = (uint8_t)((GPIOC->IDR >> 8) & 3U);
    EXTI->PR = mask;
    NVIC_SetPriority(EXTI9_5_IRQn, 4);
    NVIC_ClearPendingIRQ(EXTI9_5_IRQn);
    EXTI->IMR |= mask;
    NVIC_EnableIRQ(EXTI9_5_IRQn);
}

/* Motor 1: PC10=A, PC11=B, EXTI10/11. */
static volatile uint16_t encoder1_count;
static uint8_t encoder1_previous;
static const int8_t encoder1_step[16] = {
    0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0
};
void EXTI15_10_IRQHandler(void)
{
    const uint32_t mask = (1UL << 10) | (1UL << 11);
    uint32_t pending = EXTI->PR & mask;
    if (pending) {
        uint8_t state;
        EXTI->PR = pending;
        state = (uint8_t)((GPIOC->IDR >> 10) & 3U);
        encoder1_count = (uint16_t)(encoder1_count +
            encoder1_step[(encoder1_previous << 2) | state]);
        encoder1_previous = state;
    }
}
static void encoder1_init(void)
{
    const uint32_t mask = (1UL << 10) | (1UL << 11);
    EXTI->IMR &= ~mask;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    (void)RCC->APB2ENR;
    GPIOC->MODER &= ~((3UL << 20) | (3UL << 22));
    GPIOC->PUPDR &= ~((3UL << 20) | (3UL << 22));
    /* EXTI10 and EXTI11 select port C (2). */
    SYSCFG->EXTICR[2] = (SYSCFG->EXTICR[2] & ~0xff00UL) | 0x2200UL;
    EXTI->RTSR |= mask;
    EXTI->FTSR |= mask;
    encoder1_count = 0;
    encoder1_previous = (uint8_t)((GPIOC->IDR >> 10) & 3U);
    EXTI->PR = mask;
    NVIC_SetPriority(EXTI15_10_IRQn, 4);
    NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
    EXTI->IMR |= mask;
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

/* Dedicated TIM2: PA0=CH1, PB3=CH2, AF1. No pulses until requested.
 * The current model has a 96 MHz APB1 timer clock. 1 us ticks, 20 ms frame.
 * PB3 is reserved for steering; use SWD, not SWO/full JTAG tracing.
 */
void vehicle_servo_io(double front_us, double rear_us)
{
    static uint8_t initialized = 0;
    uint32_t channels = 0;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
        (void)RCC->APB1ENR;
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM2RST;
        RCC->APB1RSTR &= ~RCC_APB1RSTR_TIM2RST;
        TIM2->PSC = 95;
        TIM2->ARR = 19999;
        TIM2->CCR1 = 0; TIM2->CCR2 = 0;
        TIM2->CCMR1 = (6UL<<4)|(1UL<<3)|(6UL<<12)|(1UL<<11);
        TIM2->CCER = 0;
        TIM2->EGR = 1;
        alternate(GPIOA,0,1); alternate(GPIOB,3,1);
        /* Hold the lines low while their timer channels are disabled. */
        GPIOA->PUPDR = (GPIOA->PUPDR & ~3UL) | 2UL;
        GPIOB->PUPDR = (GPIOB->PUPDR & ~(3UL<<6)) | (2UL<<6);
        TIM2->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
        initialized = 1;
    }
    if (front_us >= 1000 && front_us <= 2000) {
        if (TIM2->CCR1 != (uint32_t)front_us) { TIM2->CCR1 = (uint32_t)front_us; }
        channels |= TIM_CCER_CC1E;
    } else { if (TIM2->CCR1 != 0) { TIM2->CCR1 = 0; } }
    if (rear_us >= 1000 && rear_us <= 2000) {
        if (TIM2->CCR2 != (uint32_t)rear_us) { TIM2->CCR2 = (uint32_t)rear_us; }
        channels |= TIM_CCER_CC2E;
    } else { if (TIM2->CCR2 != 0) { TIM2->CCR2 = 0; } }
    if (TIM2->CCER != channels) { TIM2->CCER = channels; }
}
void vehicle_servo_status(uint16_t *front_us, uint16_t *rear_us, uint16_t *period_us)
{
    *front_us = (TIM2->CCER & TIM_CCER_CC1E) ? (uint16_t)TIM2->CCR1 : 0;
    *rear_us = (TIM2->CCER & TIM_CCER_CC2E) ? (uint16_t)TIM2->CCR2 : 0;
    *period_us = (uint16_t)(TIM2->ARR + 1U);
}
void vehicle_io(double duty, uint8_t enable, uint16_t *e1, uint16_t *e2)
{
    static uint8_t initialized = 0;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
                         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOFEN;
        RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

        (void)RCC->APB2ENR;
        GPIOF->BSRR = (1UL << (12+16)) | (1UL << (13+16));
        GPIOF->MODER = (GPIOF->MODER & ~((3UL<<24)|(3UL<<26))) | (1UL<<24)|(1UL<<26);
        GPIOF->OTYPER &= ~((1UL<<12)|(1UL<<13));
        GPIOF->PUPDR = (GPIOF->PUPDR & ~((3UL<<24)|(3UL<<26))) | (2UL<<24)|(2UL<<26);
        RCC->APB1RSTR |= RCC_APB1RSTR_TIM4RST;
        RCC->APB1RSTR &= ~(RCC_APB1RSTR_TIM4RST);


        encoder1_init();
        encoder2_init();
        /* This project's clock setup gives TIM1 and TIM4 96 MHz each:
         * HCLK=96 MHz, APB1=/2 with timer x2, APB2=/1, TIMPRE=0.
         * Reserve the top compare count so CCR=ARR+1 for true 100% fits
         * the 16-bit registers. Both timers then run at ~1464.87 Hz.
         * Recheck both timer clocks if the CubeMX clock setup changes.
         */
        if (TIM1->ARR == 65535UL) { TIM1->ARR = 65534UL; }
        TIM4->PSC = TIM1->PSC;
        TIM4->ARR = TIM1->ARR;
        TIM4->CCR3 = 0; TIM4->CCR4 = 0;
        TIM4->CCMR2 = (6UL<<4)|(1UL<<3)|(6UL<<12)|(1UL<<11);
        TIM4->CCER = TIM_CCER_CC3E | TIM_CCER_CC4E;
        TIM4->EGR = 1;
        alternate(GPIOD,14,2); alternate(GPIOD,15,2);
        TIM4->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
        initialized = 1;
    }
    /* Signed duty: positive=RPWM, negative=LPWM. At most one is active.
     * The command layer enforces zero output for 500 ms before reversal.
     */
    if (!enable || !(duty >= -100.0 && duty <= 100.0)) {
        GPIOF->BSRR = (1UL<<28)|(1UL<<29);
        TIM4->CCR4 = 0; TIM4->CCR3 = 0;
    } else {
        if (duty >= 0.0) {
            TIM4->CCR3 = 0;
            TIM4->CCR4 = (uint32_t)((TIM4->ARR+1U)*duty/100.0);
        } else {
            TIM4->CCR4 = 0;
            TIM4->CCR3 = (uint32_t)((TIM4->ARR+1U)*(-duty)/100.0);
        }
        GPIOF->BSRR = (1UL<<12)|(1UL<<13);
    }
    /* Independent 1 ms pin sampling, regardless of EXTI service. */
    encoder2_seen |= (uint8_t)(1U << ((GPIOC->IDR >> 8) & 3U));
    current_ina219_tick();
    *e1 = encoder1_count;
    *e2 = encoder2_count;
}

/* Raw E2 input diagnostics; does not change counting, pin mapping or drive. */
uint16_t vehicle_encoder_diag(uint8_t *out)
{
    char text[48];
    unsigned state = (unsigned)((GPIOC->IDR >> 8) & 3U);
    unsigned a = encoder2_a_edges, b = encoder2_b_edges, bad = encoder2_invalid;
    unsigned config = 0;
    const uint32_t mask = (1UL << 8) | (1UL << 9);
    if ((GPIOC->MODER & ((3UL << 16) | (3UL << 18))) == 0) config |= 1;
    if ((SYSCFG->EXTICR[2] & 0xffUL) == 0x22UL) config |= 2;
    if ((EXTI->RTSR & mask) == mask) config |= 4;
    if ((EXTI->FTSR & mask) == mask) config |= 8;
    if ((EXTI->IMR & mask) == mask) config |= 16;
    if (NVIC_GetEnableIRQ(EXTI9_5_IRQn)) config |= 32;
    if (__get_PRIMASK() == 0) config |= 64;
    int n = snprintf(text, sizeof(text), " P2=%u A2=%05u B2=%05u X2=%05u G2=%u C2=%u",
                     state, a, b, bad, (unsigned)encoder2_seen, config);
    if (n < 0 || n >= (int)sizeof(text)) return 0;
    memcpy(out, text, (unsigned)n);
    return (uint16_t)n;
}
