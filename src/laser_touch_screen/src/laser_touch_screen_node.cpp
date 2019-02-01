#include "laser_touch_screen/laser_touch_screen.h"

laser_touch_screen_cls *tl_p;
laser_touch_screen_cls *lr_p;
visualization_msgs::Marker boundary_marker;
ros::Publisher boundary_pub;
void laser_touch_screen_config_callback(laser_touch_screen::laser_touch_screen_Config &config, uint32_t level)
{
    printf("============================Reconfigure===========================\r\n");
	tl_p->origin_frame_id = config.origin_frame_id;
	tl_p->screen_origin_frame_id = config.screen_origin_frame_id;
	tl_p->endpoint_dist_threshold = config.endpoint_dist_threshold;
	tl_p->with_threshold_l = config.with_threshold_l;
	tl_p->with_threshold_h = config.with_threshold_h;


	tl_p->superior_frame_id = config.top_left_laser_frame_id;
	tl_p->frame_id = config.top_left_frame_id;
	tl_p->scan_topic_name = config.top_left_scan_topic_name;
	tl_p->pub_end_point_en = config.top_left_pub_end_point_en;
	tl_p->module_namespace = config.top_left_module_namespace;
    tl_p->scan_origin_x = config.top_left_scan_origin_x;
    tl_p->scan_origin_y = config.top_left_scan_origin_y;
    tl_p->scan_origin_z = config.top_left_scan_origin_z;
    tl_p->scan_yaw = config.top_left_scan_yaw;
    tl_p->scan_pitch = config.top_left_scan_pitch;
    tl_p->scan_roll = config.top_left_scan_roll;
	tl_p->init();

	// std::cout << config.top_left_scan_yaw << std::endl;

	lr_p->origin_frame_id = config.origin_frame_id;
	lr_p->screen_origin_frame_id = config.screen_origin_frame_id;
	lr_p->endpoint_dist_threshold = config.endpoint_dist_threshold;
	lr_p->with_threshold_l = config.with_threshold_l;
	lr_p->with_threshold_h = config.with_threshold_h;

	lr_p->superior_frame_id = config.lower_right_laser_frame_id;
	lr_p->frame_id = config.lower_right_frame_id;
	lr_p->scan_topic_name = config.lower_right_scan_topic_name;
	lr_p->pub_end_point_en = config.lower_right_pub_end_point_en;
	lr_p->module_namespace = config.lower_right_module_namespace;
    lr_p->scan_origin_x = config.lower_right_scan_origin_x;
    lr_p->scan_origin_y = config.lower_right_scan_origin_y;
    lr_p->scan_origin_z = config.lower_right_scan_origin_z;
    lr_p->scan_yaw = config.lower_right_scan_yaw;
    lr_p->scan_pitch = config.lower_right_scan_pitch;
    lr_p->scan_roll = config.lower_right_scan_roll;
	lr_p->init();
}

std::mutex origin_lower_right_scan_tf_mutex;
tf::StampedTransform origin_lower_right_scan_tf;


void initialize_boundary()
{
    boundary_marker.header.stamp = ros::Time::now();
    boundary_marker.header.frame_id = tl_p->origin_frame_id;
    boundary_marker.id = 0;
    boundary_marker.type = visualization_msgs::Marker::LINE_STRIP;
    boundary_marker.color.r = 0.0;
    boundary_marker.color.g = 1.0;
    boundary_marker.color.b = 0.0;
    boundary_marker.color.a = 1.0;
    boundary_marker.scale.x = 0.05;
    boundary_marker.scale.y = 0.0;
    boundary_marker.scale.z = 0.0;
    boundary_marker.ns = "laser_touch_screen";
    boundary_marker.lifetime = ros::Duration(99);
    boundary_marker.action  = visualization_msgs::Marker::ADD;
    boundary_marker.pose.orientation.w = 1.0;

    geometry_msgs::Point point;
    std::vector<geometry_msgs::Point> point_array;

    //first point
    point.x = 0;
    point.y = 0;
    point.z = 0;

    point_array.emplace_back(point);

    //second point
    point.x = 0;
    point.y = origin_lower_right_scan_tf.getOrigin().y();
    point.z = 0;

    point_array.emplace_back(point);

    //third point
    point.x = origin_lower_right_scan_tf.getOrigin().x();
    point.y = origin_lower_right_scan_tf.getOrigin().y();
    point.z = 0;

    point_array.emplace_back(point);

    //fourth point
    point.x = origin_lower_right_scan_tf.getOrigin().x();
    point.y = 0;
    point.z = 0;
    point_array.emplace_back(point);

    //first point again to complete the box
    point.x = 0;
    point.y = 0;
    point.z = 0;
    point_array.emplace_back(point);

    boundary_marker.points = point_array;

    boundary_pub.publish(boundary_marker);
}

void get_tf()
{
    tf::TransformListener listenner;
    ros::Rate rate(50);

    while (ros::ok())
    {
        try
        {
            listenner.waitForTransform(lr_p->origin_frame_id, lr_p->superior_frame_id, ros::Time(0), ros::Duration(60.0));
            origin_lower_right_scan_tf_mutex.lock();
                listenner.lookupTransform(lr_p->origin_frame_id, lr_p->superior_frame_id, ros::Time(0), origin_lower_right_scan_tf);
            origin_lower_right_scan_tf_mutex.unlock();
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("Transform error: %s", ex.what());
        }

        // std::cout << origin_lower_right_scan_tf.getOrigin().x() << std::endl;
        // std::cout << origin_lower_right_scan_tf.getOrigin().y() << std::endl;
        // std::cout << origin_lower_right_scan_tf.getOrigin().z() << std::endl;

        tl_p->x_limit_against_origin = origin_lower_right_scan_tf.getOrigin().x();
        tl_p->y_limit_against_origin = origin_lower_right_scan_tf.getOrigin().y();
        lr_p->x_limit_against_origin = origin_lower_right_scan_tf.getOrigin().x();
        lr_p->y_limit_against_origin = origin_lower_right_scan_tf.getOrigin().y();

        // printf ("-=-=-=-=-=-=--=-=-=-=-=-=-=-=\r\n");

    initialize_boundary();
        rate.sleep();
    }
}


int main(int argc,char** argv)
{
    ros::init(argc, argv, "laser_touch_screen");

	ros::NodeHandle nh("~");

	laser_touch_screen_cls ltc_top_left;
	laser_touch_screen_cls ltc_lower_right;
	tl_p = &ltc_top_left;
	lr_p = &ltc_lower_right;

    // nh.param("origin_frame_id", ltc_top_left.origin_frame_id,                       std::string("origin"));
    // nh.param("top_left_scan_name", ltc_top_left.scan_topic_name,                    std::string("/scan1"));
    // nh.param("top_left_frame_id", ltc_top_left.frame_id,                            std::string("origin"));
    // nh.param("top_left_module_namespace", ltc_top_left.module_namespace,            std::string("top_left"));
    // nh.param("top_left_pub_end_point_en", ltc_top_left.pub_end_point_en,            true);
    // nh.param("top_left_scan_origin_x", ltc_top_left.scan_origin_x,                  0.0);
    // nh.param("top_left_scan_origin_y", ltc_top_left.scan_origin_y,                  0.0);
    // nh.param("top_left_scan_origin_z", ltc_top_left.scan_origin_z,                  0.0);
    // nh.param("top_left_scan_yaw", ltc_top_left.scan_yaw,                            0.0);
    // nh.param("top_left_scan_pitch", ltc_top_left.scan_pitch,                        0.0);
    // nh.param("top_left_scan_roll", ltc_top_left.scan_roll,                          0.0);

    // nh.param("origin_frame_id", ltc_lower_right.origin_frame_id,                    std::string("origin"));
    // nh.param("lower_right_scan_name", ltc_lower_right.scan_topic_name,              std::string("/scan2"));
    // nh.param("lower_right_frame_id", ltc_lower_right.frame_id,                      std::string("origin"));
    // nh.param("lower_right_module_namespace", ltc_lower_right.module_namespace,      std::string("lower_right"));
    // nh.param("lower_right_pub_end_point_en", ltc_lower_right.pub_end_point_en,      true);
    // nh.param("lower_right_scan_origin_x", ltc_lower_right.scan_origin_x,            0.0);
    // nh.param("lower_right_scan_origin_y", ltc_lower_right.scan_origin_y,            0.0);
    // nh.param("lower_right_scan_origin_z", ltc_lower_right.scan_origin_z,            0.0);
    // nh.param("lower_right_scan_yaw", ltc_lower_right.scan_yaw,                      0.0);
    // nh.param("lower_right_scan_pitch", ltc_lower_right.scan_pitch,                  0.0);
    // nh.param("lower_right_scan_roll", ltc_lower_right.scan_roll,                    0.0);

    dynamic_reconfigure::Server<laser_touch_screen::laser_touch_screen_Config> mc_server;
    dynamic_reconfigure::Server<laser_touch_screen::laser_touch_screen_Config>::CallbackType mc_f;
    mc_f = &laser_touch_screen_config_callback;
    mc_server.setCallback(mc_f);

	ltc_top_left.init();
	ltc_lower_right.init();

    boundary_pub = nh.advertise<visualization_msgs::Marker>("boundary",10);
    initialize_boundary();

	// std::cout << ltc_top_left.frame_id << std::endl;
	// std::cout << ltc_lower_right.frame_id << std::endl;

    ros::AsyncSpinner spinner_1(8, &ltc_top_left.cb_queue);
    spinner_1.start();

    ros::AsyncSpinner spinner_2(8, &ltc_lower_right.cb_queue);
    spinner_2.start();

	std::thread tl_pub_tf_thr(boost::bind(&laser_touch_screen_cls::pub_tf, &ltc_top_left));
	std::thread tl_get_tf_thr(boost::bind(&laser_touch_screen_cls::get_tf, &ltc_top_left));
	std::thread lr_pub_tf_thr(boost::bind(&laser_touch_screen_cls::pub_tf, &ltc_lower_right));
	std::thread lr_get_tf_thr(boost::bind(&laser_touch_screen_cls::get_tf, &ltc_lower_right));
	std::thread get_tf_thr(boost::bind(&get_tf));

	ros::spin();
	tl_pub_tf_thr.join();
	tl_get_tf_thr.join();
	lr_pub_tf_thr.join();
	lr_get_tf_thr.join();
    get_tf_thr.join();

	return 0;
}   

