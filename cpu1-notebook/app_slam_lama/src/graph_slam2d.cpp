
#include <lama/graph_slam2d.h>
#include <lama/pose3d.h>
#include <lama/sdm/occupancy_map.h>
#include <lama/image.h>
#include <lama/print.h>
#include <Eigen/StdVector>
#include <ufr.h>

#include "opencv2/opencv.hpp"

using namespace cv;
using namespace lama;

typedef struct {
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float scan_time;
    float range_min;
    float range_max;
    float ranges[2048];
} Laser;


Mat cv_map(30,30,CV_8UC1);

Pose3D getSensorPose(std::string frame_id) {
    return Pose3D();
}

bool getOdometry(Pose2D& odom) {
    odom = Pose2D(0,0,0);
    return true;
}


bool OccupancyMsgFromOccupancyMap(GraphSlam2D* slam2d_) {

    const OccupancyMap* occ = slam2d_->generateOccupancyMap(true).get();

    auto map = occ;
    Vector3ui imin, imax;
    map->bounds(imin, imax);

    unsigned int width = imax(0) - imin(0);
    unsigned int height= imax(1) - imin(1);

    if (width == 0 || height == 0)
        return false;

    Image image;
    image.alloc(width, height, 1);
    image.fill(0xff);

    map->visit_all_cells([&image, &map, &imin](const Vector3ui& coords){
        Vector3ui adj_coords = coords - imin;

        if (map->isFree(coords))
            image(adj_coords(0), adj_coords(1)) = 0;
        else if (map->isOccupied(coords))
            image(adj_coords(0), adj_coords(1)) = 255;
    });

    
    printf("img %d %d\n", width, height);
    cv_map = Mat(height, width, CV_8UC1);
    memcpy(&cv_map.data[0], image.data.get(), width*height);
/*
    

    msg.info.width = width;
    msg.info.height = height;
    msg.info.resolution = map->resolution;

    Vector3d pos = map->m2w(imin);
    msg.info.origin.position.x = pos.x();
    msg.info.origin.position.y = pos.y();
    msg.info.origin.position.z = 0;
    msg.info.origin.orientation = tf::createQuaternionMsgFromYaw(0);
*/

    return true;
}


int main() {
    link_t* laser_sub = ufr_subscriber_env("UFR_SCAN");
    link_t* odom_sub = ufr_subscriber_env("UFR_ODOM");

    //
    GraphSlam2D::Options options;
    GraphSlam2D* slam2d_ = new GraphSlam2D(options);

    //
    double tmp = 0.0;
    Vector2d pos(0,0); 
    Pose2D prior(pos, tmp);
    slam2d_->Init(prior);

    const float min_range_ = 0.15;
    const float max_range_ = 12.0;
    const int beam_step_ = 1;

    float odom_x, odom_y, odom_th;
    float scan_angle_min;
    float scan_angle_max;
    float scan_angle_inc;
    float scan_time_inc;
    float scan_range_min;
    float scan_range_max;
    float scan_time;
    int scan_ranges_size;
    float scan_ranges[4096];

    Pose3D sensor_origin(0,0,0,0,0,0);
    printf("Main loop\n");

    // Main Loop
    while ( ufr_loop_ok() ) {
        imshow("mapa", cv_map);
        waitKey(1);

        if ( ufr_recv_async(laser_sub) ) {
            // printf("lidar\n");
            ufr_get(laser_sub, "%f %f", &scan_angle_min, &scan_angle_max);
            ufr_get(laser_sub, "%f %f", &scan_angle_inc, &scan_time_inc);
            ufr_get(laser_sub, "%f", &scan_time);
            ufr_get(laser_sub, "%f %f", &scan_range_min, &scan_range_max);
            scan_ranges_size = ufr_get_af32(laser_sub, scan_ranges, 4096);
        }

        if ( ufr_recv_async(odom_sub) ) {
            ufr_get(odom_sub, "%f %f %f", &odom_x, &odom_y, &odom_th);
            // printf("odom %f %f %f\n", odom_x, odom_y, odom_th);
        }

        // Check if the updated is needed
        Pose2D odom(odom_x, odom_y, odom_th);
        /* const bool update = slam2d_->enoughMotion(odom);
        if ( !update ) {
            continue;
        }*/

        printf("calculando\n");

        //
        float max_range;
        if (max_range_ == 0.0 || max_range_ > scan_range_max)
            max_range = scan_range_max;
        else
            max_range = max_range_;

        //
        float min_range;
        if (min_range_ == 0 || min_range_ < scan_range_min)
            min_range = scan_range_min;
        else
            min_range = min_range_;

        //
        PointCloudXYZ::Ptr cloud(new PointCloudXYZ);
        cloud->sensor_origin_ = sensor_origin.xyz();
        cloud->sensor_orientation_ = Quaterniond(sensor_origin.state.so3().matrix());

        //
        cloud->points.reserve(scan_ranges_size);
        for(size_t i = 0; i < scan_ranges_size; i += beam_step_){
            const double range = scan_ranges[i];

            if (not std::isfinite(range))
                continue;

            if (range >= max_range || range <= min_range)
                continue;
            
            const double x = range * std::cos(scan_angle_min+(i*scan_angle_inc));
            const double y = range * std::sin(scan_angle_min+(i*scan_angle_inc));
            cloud->points.push_back( Eigen::Vector3d(x,y,0) );
        }

        //
        double timestamp = 0;
        slam2d_->update(cloud, odom, timestamp);
        Pose2D pose = slam2d_->getPose();

        printf("slam-pose %f %f %f\n", pose.x(), pose.y(), pose.rotation());
        OccupancyMsgFromOccupancyMap(slam2d_);

        
    }

    // end
    ufr_close(laser_sub);
    return 0;
}