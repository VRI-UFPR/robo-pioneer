#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/device.h>
#include <webots/robot.h>
#include <webots/lidar.h>


#include "lidar.h"


Lidar* lidar_new(int time_step) {
    Lidar* lidar = malloc(sizeof(Lidar));
    const bool is_ok = lidar_init(lidar, time_step);
    return (is_ok) ? lidar : NULL;
}


bool lidar_init(Lidar* lidar, int time_step) {
    bool is_ok = false;
    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        if ( type == 48 ) {
            printf("Lidar inicializado %s\n", name);
            wb_lidar_enable(tag, time_step);
            lidar->tag = tag;
            is_ok = true;
        }
    }

    return is_ok;
}


void lidar_step_and_publish(Lidar* lidar, link_t* link) {
    if ( lidar == NULL ) {
        return;
    }

    
    const float range_min = 0.0;
    const float range_max = wb_lidar_get_max_range(lidar->tag);

    const float angle_min = 0.0;
    const float angle_max = wb_lidar_get_fov(lidar->tag);
    const float angle_increment = angle_max / wb_lidar_get_horizontal_resolution(lidar->tag);
    const float scan_time = 0;
    const float time_increment = 0;

    // printf("%f %f %f\n", lidar->x, lidar->y, lidar->th);
    ufr_put(link, "%f %f %f", angle_min, angle_max, angle_increment);
    ufr_put(link, "%f %f %f %f", time_increment, scan_time, range_min, range_max);


    const float* range_ptr = wb_lidar_get_range_image(lidar->tag);
    const int range_size = wb_lidar_get_horizontal_resolution(lidar->tag);
    ufr_put_af32(link, range_ptr, range_size);
    ufr_send(link);
}