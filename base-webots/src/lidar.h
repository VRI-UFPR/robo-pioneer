#include <ufr.h>
#include <webots/device.h>


typedef struct {
    WbDeviceTag tag;
} Lidar;


Lidar* lidar_new(int time_step);
bool  lidar_init(Lidar* lidar, int time_step);
void lidar_step_and_publish(Lidar* lidar, link_t* link);
