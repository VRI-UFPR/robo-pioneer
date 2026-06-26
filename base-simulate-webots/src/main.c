#include <ufr.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/robot.h>


#include "motor.h"
#include "odom.h"
#include "camera.h"
#include "lidar.h"


const int g_time_step = 100;


int main() {
    // Inicio
    wb_robot_init();

    // Show Information
    const char* model = wb_robot_get_model();
    printf("Robot model: %s\n", model);
    const int n_devices = wb_robot_get_number_of_devices();
    for(int i=0; i<n_devices; i++) {
        WbDeviceTag tag = wb_robot_get_device_by_index(i);
        const char *name = wb_device_get_name(tag);
        WbNodeType type = wb_device_get_node_type(tag);
        printf("Device #%d, name = \"%s\", type = %d\n", i, name, type);
    }

    Motor* motor = motor_new(g_time_step);
    Odom* odom = odom_new(g_time_step);
    Lidar* lidar = lidar_new(g_time_step);
    Camera* camera = camera_new(g_time_step);


    link_t* lnk_cmdvel = ufr_subscriber_env("UFR_CMDVEL");
    link_t* lnk_odom   = ufr_publisher_env("UFR_ODOM");
    link_t* lnk_lidar  = ufr_publisher_env("UFR_SCAN");
    // link_t lnk_camera = ufr_publisher("@new mqtt @coder msgpack @host vriufpr.ddns.net @topic /pioneer/camera");


    // Loop principal
    while( ufr_loop() ) {
        wb_robot_step(g_time_step);

        // Verifica se recebeu mensagem no cmd_vel
        if ( ufr_recv_async(lnk_cmdvel) ) {
            float vel, rotvel;
            ufr_get(lnk_cmdvel, "%f %f", &vel, &rotvel);
            printf("motor %f %f\n", vel, rotvel);
            motor_set_speed(motor, vel, rotvel);
        }

        // Odometria
        odom_step_and_publish(odom, lnk_odom);
        lidar_step_and_publish(lidar, lnk_lidar);
        // camera_step_and_publish(camera, lnk_lidar);
    }

    // Fim
    printf("Fim\n");
    wb_robot_cleanup();
    return 0;
}