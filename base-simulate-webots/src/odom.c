
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/device.h>
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>

#include "odom.h"


Odom* odom_new(int time_step) {
    Odom* odom = malloc(sizeof(Odom));
    const bool is_ok = odom_init(odom, time_step);
    return (is_ok) ? odom : NULL;
}


bool odom_init(Odom* odom, int time_step) {
    bool is_ok = false;
    odom->is_first_read = true;
    odom->wheel_left_last = 0;
    odom->wheel_right_last = 0;

    odom->x = 0;
    odom->y = 0;
    odom->th = 0;

    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        if ( type == WB_NODE_POSITION_SENSOR ) {
            if ( strstr(name, "left") != NULL ) {
                printf("Sensor inicializado %s\n", name);
                odom->left = wb_robot_get_device(name);
                // const int time_step = ufr_gtw_webots_get_time_step();
                wb_position_sensor_enable(odom->left, time_step);
            } else if ( strstr(name, "right") != NULL ) {
                printf("Sensor inicializado %s\n", name);
                odom->right = wb_robot_get_device(name);
                // const int time_step = ufr_gtw_webots_get_time_step();
                wb_position_sensor_enable(odom->right, time_step);
            }
            is_ok = true;
        }
    }

    return is_ok;
}


void odom_step_and_publish(Odom* odom, link_t* link) {
    if ( odom == NULL ) {
        return;
    }

    odom_step(odom);
    printf("%f %f %f\n", odom->x, odom->y, odom->th*180.0/3.141592);
    ufr_put(link, "%f %f %f\n", odom->x, odom->y, odom->th);
}

void odom_step(Odom* odom) {
    // get new read
    const double wheel_left = wb_position_sensor_get_value(odom->left);
    const double wheel_right = wb_position_sensor_get_value(odom->right);

    // case is not first read
    if ( odom->is_first_read == false ) {
        const double diff_left = wheel_left - odom->wheel_left_last;
        const double diff_right = wheel_right - odom->wheel_right_last;
        odom->x += ((diff_left + diff_right) * cos(odom->th)) / 100.0;
        odom->y += ((diff_left + diff_right) * sin(odom->th)) / 100.0;
        odom->th += (diff_left - diff_right) * 0.3;
    }

    // update last read
    odom->wheel_left_last = wheel_left;
    odom->wheel_right_last = wheel_right;
    odom->is_first_read = false;
}