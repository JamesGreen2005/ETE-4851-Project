#ifndef DUAL_MOTOR_IO_H
#define DUAL_MOTOR_IO_H
#include <stdint.h>
void vehicle_io(double duty, uint8_t enable, uint16_t *encoder1, uint16_t *encoder2);
void vehicle_servo_io(double front_us, double rear_us);
void vehicle_servo_status(uint16_t *front_us, uint16_t *rear_us, uint16_t *period_us);
#endif

uint16_t vehicle_encoder_diag(uint8_t *out);
