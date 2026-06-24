#pragma once

#include <webots/robot.h>

#define MAX_MOTORS 4

typedef struct {
    uint8_t count;                     // 1 motor de cada lado ou 2 motores de cada lado
    WbDeviceTag left[MAX_MOTORS];
    WbDeviceTag right[MAX_MOTORS];
} Motor;


Motor* motor_new(int time_step);
bool motor_init(Motor* motor, int time_step);
void motor_set_speed(Motor* motor, double vel, double rotvel);