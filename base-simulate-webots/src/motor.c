#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>


#include <webots/device.h>
#include <webots/robot.h>
#include <webots/motor.h>

#include "motor.h"


Motor* motor_new(int time_step) {
    Motor* motor = malloc(sizeof(Motor));
    const bool is_ok = motor_init(motor, time_step);
    return (is_ok) ? motor : NULL;
}

bool motor_init(Motor* motor, int time_step) {
    // Init
    char const* dev_left_default[MAX_MOTORS];
    char const* dev_right_default[MAX_MOTORS];
    for (uint8_t i=0; i<MAX_MOTORS; i++) {
        dev_left_default[i] = NULL;
        dev_right_default[i] = NULL;
    }

    // Search by motors
    uint8_t left_count = 0;
    uint8_t right_count = 0;
    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        if ( type == WB_NODE_ROTATIONAL_MOTOR ) {
            if ( strstr(name, "left") != NULL ) {
                printf("Motor inicializado %s\n", name);
                dev_left_default[left_count] = name;
                left_count += (left_count<MAX_MOTORS) ? 1 : 0;

            } else if ( strstr(name, "right") != NULL ) {
                printf("Motor inicializado %s\n", name);
                dev_right_default[right_count] = name;
                right_count += (right_count<MAX_MOTORS) ? 1 : 0;
            }
        }
    }

    // Check if there are same number of motors each side
    if ( left_count != right_count ) {
        printf("error1\n");
        return false;
    }

    if ( left_count > MAX_MOTORS ) {
        printf("error2\n");
        return false;
    }

    // Inicializa os motores
    const uint8_t count = left_count;
    motor->count = count;
    for (uint8_t i=0; i<count; i++) {
        motor->left[i] = wb_robot_get_device( dev_left_default[i] );
        motor->right[i] = wb_robot_get_device( dev_right_default[i] );

        // Inicialize the motors
        wb_motor_set_position(motor->left[i], INFINITY);
        wb_motor_set_position(motor->right[i], INFINITY);
        wb_motor_set_velocity(motor->left[i], 0.0);
        wb_motor_set_velocity(motor->right[i], 0.0);
    }

    return true;
}

void motor_set_speed(Motor* motor, double vel, double rotvel) {
    const double speed_left = vel - (rotvel * 0.125); // HALF_DISTANCE_BETWEEN_WHEELS
    const double speed_right = vel + (rotvel * 0.125); // HALF_DISTANCE_BETWEEN_WHEELS

    // Send the speed to the WeBots
    for ( uint8_t i=0; i<motor->count; i++ ) {
        wb_motor_set_velocity(motor->left[i], speed_left);
        wb_motor_set_velocity(motor->right[i], speed_right);
    }
}
