#ifndef CURRENT_INA219_H
#define CURRENT_INA219_H
#include <stdint.h>
/* Tick once per 1 ms base task. No waits, interrupts, or motor commands. */
void current_ina219_tick(void);
/* Raw shunt units = 10 uV; bus units = mV. Status 0=offline,1=OK,2=range. */
void current_ina219_status(int16_t *sh1, uint16_t *bv1, uint8_t *st1,
                           int16_t *sh2, uint16_t *bv2, uint8_t *st2);
#endif
