#ifndef HEADER_H
#define HEADER_H

#include <Arduino.h>
#include <MPU6050_light.h>
#include "debug_in_color.h"

/*----const-------*/
const unsigned int ANALOG_WRITE_RANGE = 4096;
const unsigned int ANALOG_WRITE_FREQ = 20000;

/*----pins-------*/
#define PIN_FUNK_CS 21
#define PIN_RADIO_MISO 16
#define PIN_RADIO_MOSI 19
#define PIN_RADIO_SCK 18
#define PIN_RADIO_GDO0 20
#define PIN_RADIO_GDO2 17

#define PIN_BATT 29
#define PIN_MOT0 10 //PWM 5A
#define PIN_MOT1 12 //PWM 6A
#define PIN_MOT2 24 //PWM 4A 
#define PIN_MOT3 1 //PWM 0B

#define PIN_MPU_SCL 3
#define PIN_MPU_SDA 2

#define BATVOLTMIN 2.2 // at this voltage the porzessor disables all Motors

/*utilitifunctions*/
extern char mpu_print_enable;
extern int BatReadIntervall;
void print_batVolt();
void serial_commands();
void printMemoryUse();
void battery_uv_protection();

/*i2c scan*/
void scan_wire();

/*mpu*/
extern MPU6050 mpu;
void mpu_setup();
void mpu_print();

/*Radio*/
void radio_calibrate();
void radio_setup();
void radio_recieve_buffer();
void radio_read();

/*pid*/
extern float kp_x;
extern float ki_x;
extern float kd_x;
void pid_update();
void set_motor(uint8_t Motor_nr, float duty);

#endif