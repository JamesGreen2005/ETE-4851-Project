#include "current_ina219.h"
#include "stm32f7xx.h"

/* Dedicated I2C1: PB8=SCL/PB9=SDA, AF4, open drain, external 3.3 V
 * pullups supplied by breakouts. Addresses 0x40 and 0x41. Other I2C buses
 * and the UART IMU are untouched. Fixed existing APB1 clock: 48 MHz.
 * One state transition per tick: a missing/stuck sensor never blocks control.
 * Reconfigure each sample so a brownout/reset is recovered automatically.
 * Raw shunt readings avoid dependence on the INA219 calibration register.
 */
static uint32_t ticks, deadline, next_sample, stamp[2];
static uint8_t initialized, state, sensor, stage, pos, count, receiving;
static uint8_t tx[3], rx[2], status[2];
static int16_t shunt[2], pending_shunt;
static uint16_t bus_mv[2];

static void reset_bus(void)
{
    RCC->APB1RSTR |= RCC_APB1RSTR_I2C1RST;
    RCC->APB1RSTR &= ~RCC_APB1RSTR_I2C1RST;
    /* PRESC=4: 104.17 ns ticks. SCL low=5.21 us, high=5.21 us;
     * SCLDEL=1.25 us, SDADEL=0.208 us. Conservative <100 kHz with
     * analog filtering and <=1 us rise time. Recompute if clocks change. */
    I2C1->TIMINGR = (4UL<<28)|(11UL<<20)|(2UL<<16)|(49UL<<8)|49UL;
    I2C1->CR1 = I2C_CR1_PE;
}
static void finish_sensor(void)
{
    sensor ^= 1U;
    stage = 0;
    state = 0;
    next_sample = ticks + (sensor == 0 ? 50U : 2U);
}
static void failed(void)
{
    status[sensor] = 0;
    reset_bus();
    finish_sensor();
}
static void begin(void)
{
    receiving = 0;
    pos = 0;
    tx[0] = stage == 0 ? 0 : stage; /* configuration, shunt, bus */
    tx[1] = 0x39; tx[2] = 0x9F; /* 32 V range, +/-320 mV, 12 bit, continuous */
    /* The chip's 32 V ADC setting does NOT extend its 26 V bus rating. */
    count = stage == 0 ? 3 : 1;
    I2C1->ICR = I2C_ICR_NACKCF|I2C_ICR_STOPCF|I2C_ICR_BERRCF|I2C_ICR_ARLOCF|I2C_ICR_OVRCF;
    I2C1->CR2 = ((0x40UL+sensor)<<1)|((uint32_t)count<<16)|
        (stage == 0 ? I2C_CR2_AUTOEND : 0)|I2C_CR2_START;
    deadline = ticks + 20U;
    state = 1;
}
void current_ina219_tick(void)
{
    uint32_t flags;
    ++ticks;
    if (!initialized) {
        RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
        RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
        (void)RCC->APB1ENR;
        RCC->DCKCFGR2 &= ~RCC_DCKCFGR2_I2C1SEL; /* APB1 */
        GPIOB->MODER = (GPIOB->MODER & ~((3UL<<16)|(3UL<<18)))|(2UL<<16)|(2UL<<18);
        GPIOB->OTYPER |= (1UL<<8)|(1UL<<9);
        GPIOB->PUPDR &= ~((3UL<<16)|(3UL<<18));
        GPIOB->OSPEEDR |= (3UL<<16)|(3UL<<18);
        GPIOB->AFR[1] = (GPIOB->AFR[1] & ~0xFFUL)|0x44UL;
        reset_bus();
        initialized = 1;
        next_sample = ticks + 100U;
        return;
    }
    if (state == 0) {
        if ((int32_t)(ticks-next_sample) >= 0) {
            if (I2C1->ISR & I2C_ISR_BUSY) failed();
            else begin();
        }
        return;
    }
    flags = I2C1->ISR;
    if ((flags & (I2C_ISR_NACKF|I2C_ISR_BERR|I2C_ISR_ARLO|I2C_ISR_OVR)) ||
        (int32_t)(ticks-deadline) >= 0) { failed(); return; }
    if (!receiving && (flags & I2C_ISR_TXIS) && pos < count) {
        I2C1->TXDR = tx[pos++];
        return;
    }
    if (stage != 0 && !receiving && pos == count && (flags & I2C_ISR_TC)) {
        receiving = 1; pos = 0;
        I2C1->CR2 = ((0x40UL+sensor)<<1)|(2UL<<16)|I2C_CR2_RD_WRN|
            I2C_CR2_AUTOEND|I2C_CR2_START;
        return;
    }
    if (receiving && (flags & I2C_ISR_RXNE) && pos < 2) {
        rx[pos++] = (uint8_t)I2C1->RXDR;
        return;
    }
    if (flags & I2C_ISR_STOPF) {
        I2C1->ICR = I2C_ICR_STOPCF;
        if (pos != count && !receiving) { failed(); return; }
        if (receiving && pos != 2) { failed(); return; }
        if (stage == 1) pending_shunt = (int16_t)(((uint16_t)rx[0]<<8)|rx[1]);
        if (stage == 2) {
            uint16_t raw = ((uint16_t)rx[0]<<8)|rx[1];
            if (!(raw & 2U)) { failed(); return; } /* conversion not ready */
            shunt[sensor] = pending_shunt;
            bus_mv[sensor] = (raw>>3)*4U;
            status[sensor] = pending_shunt >= 32000 || pending_shunt <= -32000 ? 2 : 1;
            stamp[sensor] = ticks;
            finish_sensor();
        } else {
            ++stage; state = 0;
            next_sample = ticks + (stage == 1 ? 3U : 1U);
        }
    }
}
void current_ina219_status(int16_t *sh1, uint16_t *bv1, uint8_t *st1,
                           int16_t *sh2, uint16_t *bv2, uint8_t *st2)
{
    *sh1 = shunt[0]; *bv1 = bus_mv[0];
    *sh2 = shunt[1]; *bv2 = bus_mv[1];
    *st1 = ticks-stamp[0] <= 250U ? status[0] : 0;
    *st2 = ticks-stamp[1] <= 250U ? status[1] : 0;
}
