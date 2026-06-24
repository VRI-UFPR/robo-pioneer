#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/device.h>
#include <webots/robot.h>
#include <webots/camera.h>

#include "camera.h"


Camera* camera_new(int time_step) {
    Camera* camera = malloc(sizeof(Camera));
    const bool is_ok = camera_init(camera, time_step);
    return (is_ok) ? camera : NULL;
}


bool camera_init(Camera* camera, int time_step) {
    bool is_ok = false;
    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        if ( type == 38 ) {
            printf("Camera inicializada %s\n", name);
            camera->tag = tag;
            wb_camera_enable(tag, time_step);
            is_ok = true;
        }
    }
    return is_ok;
}


void camera_step_and_publish(Camera* camera, link_t* link) {
    if ( camera == NULL ) {
        return;
    }

    // printf("%f %f %f\n", camera->x, camera->y, camera->th);
    // ufr_put(link, "%f %f %f\n", camera->x, camera->y, camera->th);
}