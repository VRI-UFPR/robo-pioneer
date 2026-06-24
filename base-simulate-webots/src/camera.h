#include <ufr.h>
#include <webots/device.h>


typedef struct {
    WbDeviceTag tag;
} Camera;


Camera* camera_new(int time_step);
bool  camera_init(Camera* cam, int time_step);
void camera_step_and_publish(Camera* cam, link_t* link);
