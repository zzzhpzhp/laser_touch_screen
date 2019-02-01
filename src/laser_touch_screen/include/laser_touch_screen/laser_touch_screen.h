#ifndef __laser_touch_screen_H__
#define __laser_touch_screen_H__

#include <ros/ros.h>

#include <tf/tf.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <serial/serial.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <angles/angles.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include "geometry_msgs/PoseArray.h"
#include <visualization_msgs/Marker.h>
#include <dynamic_reconfigure/server.h>

#include <laser_touch_screen/laser_touch_screen_Config.h>

#include <vector>
#include <iostream>
#include <iostream>
#include <cmath>
#include <math.h>
#include <stdlib.h>
#include <unistd.h>
#include <vector>
#include <time.h>
#include <algorithm>
#include <map>
#include <utility>
#include <thread>
#include <chrono>
#include <functional>
#include <atomic>
#include <mutex>
#include <queue>
#include <fcntl.h>

#include <ros/callback_queue.h>

#include <opencv2/highgui/highgui.hpp>  
#include "opencv2/opencv.hpp" // If include this, could't include linux/input.h

#include <laser_touch_screen/People.h>
#include <laser_touch_screen/Person.h>
#include <laser_touch_screen/PersonStamped.h>
#include <laser_touch_screen/PositionMeasurement.h>
#include <laser_touch_screen/PositionMeasurementArray.h>

#ifndef PI
#define PI            3.1415926
#endif

#ifndef DEG2RAD
#define DEG2RAD(x)       ((x) / 180.0 * PI)
#endif

#ifndef RAD2DEG
#define RAD2DEG(x)       ((x) / PI * 180.0)
#endif



class laser_touch_screen_cls
{
public:
	std::string scan_topic_name                      = "/scan1";
	std::string frame_id                             = "scan";
	std::string superior_frame_id                    = "origin";
	std::string origin_frame_id                      = "origin";
	std::string screen_origin_frame_id               = "screen_origin";
	std::string module_namespace                     = "laser_touch_screen";

    bool pub_end_point_en                            = false;

    double endpoint_dist_threshold                   = 0.1; 
    double with_threshold_l                          = 0.03;
    double with_threshold_h                          = 0.15;

    double scan_origin_x                             = 0.0;
    double scan_origin_y                             = 0.0;
    double scan_origin_z                             = 0.0;
    double scan_yaw                                  = 0.0;
    double scan_pitch                                = 0.0;
    double scan_roll                                 = 0.0;

    double x_limit_against_origin;
    double y_limit_against_origin;
	laser_touch_screen_cls();
	~laser_touch_screen_cls();
	ros::CallbackQueue cb_queue;
    void init();
    void get_tf();
    void pub_tf();

private:


    double x_against_origin;
    double y_against_origin;

    double angle_step_size;
    long lidar_point_num;
	tf::TransformBroadcaster tf_br;
    std::mutex origin_scan_tf_mutex;
    std::mutex superior_scan_tf_mutex;
    tf::StampedTransform superior_scan_tf;
    tf::StampedTransform origin_scan_tf;

    uint16_t leg_array_cnt = 0;
    sensor_msgs::LaserScan scan;
    ros::Subscriber scan_sub;
    ros::Publisher  scan_pub;
    ros::Publisher  vis_pub;
    ros::Publisher  leg_pub;
    laser_touch_screen::PositionMeasurementArray leg_array;

    bool leg_array_update = false;

    const int ANGLE = 0;
    const int DISTANCE = 1;

    void scan_cb(const sensor_msgs::LaserScan::ConstPtr& msg);
    void pub_vis(ros::Publisher& publisher, double x, double y, double z, double r, double g, double b, uint16_t id);
    void end_point_detector(ros::Publisher& mark_publisher, uint8_t disp_en, double point[][2], uint16_t num, double point_out[], uint16_t &cnt_out);
    void tf_transform(tf::StampedTransform& transform, double &x, double &y, double &z);
    void get_cord_against_origin(double &x, double &y);
};

#endif