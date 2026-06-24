#include <ufr.h>
#include <webots/device.h>


typedef struct {
    WbDeviceTag left;
    WbDeviceTag right;
    double wheel_left_last, wheel_right_last;
    double x,y,th;
    bool is_first_read;
} Odom;


Odom* odom_new(int time_step);
bool odom_init(Odom* odom, int time_step);
void odom_step_and_publish(Odom* odom, link_t* link);
void odom_step(Odom* odom);