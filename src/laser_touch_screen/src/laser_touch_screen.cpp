
/*************************************************************************
 *
 * RENAISSANCE ROBOT LLC CONFIDENTIAL
 * __________________
 *
 *  [2017] RENAISSANCE ROBOT LLC
 *  All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Renaissance Robot LLC and its suppliers, if any. The intellectual and
 * technical concepts contained herein are proprietary to Renaissance Robot LLC
 * and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law.
 *
 * Dissemination of this information or reproduction of this material is strictly
 * forbidden unless prior written permission is obtained from Renaissance Robot LLC.
 * 
 * @file:
 * @author: huangping.zhong   Mail: huangping.zhong@forwardx.ai
 * @version: v2.0
 * @brief:
 *************************************************************************/
#include "laser_touch_screen/laser_touch_screen.h"

laser_touch_screen_cls::laser_touch_screen_cls()
{
}

laser_touch_screen_cls::~laser_touch_screen_cls()
{
}

void laser_touch_screen_cls::init()
{
    ros::NodeHandle nh(module_namespace);

    scan_sub = nh.subscribe<sensor_msgs::LaserScan>(scan_topic_name, 1, boost::bind(&laser_touch_screen_cls::scan_cb, this, _1));
    scan_pub = nh.advertise<sensor_msgs::LaserScan>("scan", 10);
    vis_pub = nh.advertise<visualization_msgs::Marker>("middle_point", 1);
    leg_pub = nh.advertise<laser_touch_screen::PositionMeasurementArray>("leg_tracker_measurements", 1);

    tf::Quaternion q;
    superior_scan_tf.setOrigin(tf::Vector3(scan_origin_x, scan_origin_y, scan_origin_z));
    q.setRPY(scan_roll, scan_pitch, scan_roll);
    superior_scan_tf.setRotation(q);

    nh.setCallbackQueue(&cb_queue);
}

/**
 * @brief 
 * @param
 * @retval
 */
void laser_touch_screen_cls::pub_vis(ros::Publisher &publisher, double x, double y, double z, double r, double g, double b, uint16_t id)
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = "lts";
    marker.id = id;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.lifetime = ros::Duration(0.3);
    //only if using a MESH_RESOURCE marker type:
    //marker.mesh_resource = "package://pr2_description/meshes/base_v0/base.dae";

    publisher.publish(marker);
}

/**
 * @brief 
 * @param
 * @retval
 */
void laser_touch_screen_cls::end_point_detector(ros::Publisher &mark_publisher, uint8_t disp_en, double point[][2], uint16_t num, double point_out[], uint16_t &cnt_out)
{
    for (uint16_t i = 0; i < num - 1; i++)
    {
        double a = point[i][DISTANCE];
        double b = point[i + 1][DISTANCE];
        double a_ang = point[i][ANGLE];
        double b_ang = point[i + 1][ANGLE];
        double dif_ang = (b_ang - a_ang) * angle_step_size;
        double distance = sqrt(pow(a, 2) + pow(b, 2) - 2 * a * b * cos(fabs(dif_ang)));
        //printf ("distance: %f\r\n", distance);
        // Find the brake point according to the distance beween two point.
        if (distance > endpoint_dist_threshold)
        {
            uint16_t index;
            double x, y;

            // Save the end point and display.
            index = i;
            if (disp_en == true)
            {
                double temp = point[index][ANGLE] * angle_step_size + scan.angle_min;
                x =  point[index][DISTANCE] * cos(temp);
                y =  point[index][DISTANCE] * sin(temp);
                // printf ("%lf %lf\r\n", x, y);
                pub_vis(mark_publisher, x, y, 0, 0, 1, 0, cnt_out);
            }
            point_out[cnt_out] = index;
            cnt_out++;

            index = i + 1;
            if (disp_en == true)
            {
                double temp = point[index][ANGLE] * angle_step_size + scan.angle_min;
                x =  point[index][DISTANCE] * cos(temp);
                y =  point[index][DISTANCE] * sin(temp);
                // printf ("%lf %lf %lf %lf\r\n", x, y, temp, point[index][DISTANCE]);
                pub_vis(mark_publisher, x, y, 0, 0, 1, 0, cnt_out);
            }
            point_out[cnt_out] = index;
            cnt_out++;

            i++;
        }
    }
}

/**
 * @brief 
 * @param
 * @retval
 */
void laser_touch_screen_cls::scan_cb(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan.header = msg->header;

    scan.angle_min = msg->angle_min;
    scan.angle_max = msg->angle_max;
    scan.range_min = msg->range_min;
    scan.range_max = msg->range_max;
    scan.angle_increment = msg->angle_increment;
    scan.time_increment = msg->time_increment;
    scan.scan_time = msg->scan_time;

    lidar_point_num = msg->ranges.size();
    // std::cout << lidar_point_num << std::endl;

    angle_step_size = msg->angle_increment;
    // printf("step size %lf \r\n", angle_step_size);
    // std::cout << angle_step_size << msg->angle_increment << std::endl;

    double laser_ranges[lidar_point_num][2] = {0};
    double laser_output_step1[lidar_point_num][2] = {0};
    uint16_t laser_ranges_cnt = 0;
    uint16_t laser_output_step1_cnt = 0;
    uint16_t laser_output_step2[lidar_point_num] = {0};
    uint16_t laser_output_step2_cnt = 0;
    //  msg->angle_min + i *  angle_step_size + tf::getYaw( forward_laser_tf.getRotation() );

    double lidar_data_temp[lidar_point_num] = {0};
    for (uint16_t i = 0; i < lidar_point_num; i++)
    {
        if (std::isnan(msg->ranges[i]) == false)
        {
            lidar_data_temp[i] = msg->ranges[i];
        }
    }

    scan.ranges.resize(lidar_point_num);

    for (uint16_t i = 0; i < lidar_point_num; i++)
    {
        if (lidar_data_temp[i] != 0)
        {
            laser_ranges[laser_ranges_cnt][ANGLE] = double(i);
            // std::cout << double(i) << std::endl;
            // std::cout << (i) << std::endl;
            // std::cout << lidar_point_num << std::endl;
            laser_ranges[laser_ranges_cnt][DISTANCE] = lidar_data_temp[i];
            laser_ranges_cnt++;
        }
    }

    //printf ("laser_ranges_cnt: %d\r\n", laser_ranges_cnt);

    double brake_point[lidar_point_num] = {0};
    uint16_t brake_point_cnt = 0;
    end_point_detector(vis_pub, pub_end_point_en, laser_ranges, laser_ranges_cnt, brake_point, brake_point_cnt);

    //printf ("brake_point_cnt: %d\r\n", brake_point_cnt);

    for (uint16_t i = 0; i < brake_point_cnt - 1; i++)
    {
        uint16_t temp_l, temp_h;
        temp_l = brake_point[i];
        temp_h = brake_point[i + 1];
        // Find real line segment, real segment's point number must greater than 1.
        if (temp_h - temp_l > 1)
        {
            double a = laser_ranges[temp_l][DISTANCE];
            double b = laser_ranges[temp_h][DISTANCE];
            double a_ang = laser_ranges[temp_l][ANGLE];
            double b_ang = laser_ranges[temp_h][ANGLE];
            double dif_ang = (b_ang - a_ang) * angle_step_size;
            double length = sqrt(pow(a, 2) + pow(b, 2) - 2 * a * b * cos(fabs(dif_ang)));
            //printf ("length: %f\r\n", length);

            if (length >= with_threshold_l && length <= with_threshold_h)
            {
                for (uint16_t j = temp_l; j <= temp_h; j++)
                {
                    laser_output_step1[laser_output_step1_cnt][ANGLE] = laser_ranges[j][ANGLE];
                    laser_output_step1[laser_output_step1_cnt][DISTANCE] = laser_ranges[j][DISTANCE];
                    laser_output_step1_cnt++;
                }
                // Save the selected segment's endpoint index.
                laser_output_step2[laser_output_step2_cnt++] = temp_l;
                laser_output_step2[laser_output_step2_cnt++] = temp_h;
            }
        }
    }

    //printf ("laser_output_step1_cnt: %d\r\n", laser_output_step1_cnt);
    //printf ("laser_output_step2_cnt: %d\r\n", laser_output_step2_cnt);
    //printf ("endpoint_dist_threshold %f y_limit_ %f with_thr_l %f with_thr_h %f\r\n", endpoint_dist_threshold, y_limit_, with_threshold_l, with_threshold_h);

    for (uint16_t i = 0; i < laser_output_step1_cnt; i++)
    {
        scan.ranges[laser_output_step1[i][ANGLE]] = laser_output_step1[i][DISTANCE];
    }

    scan_pub.publish(scan);

    std::vector<laser_touch_screen::PositionMeasurement> legs;
    for (uint16_t i = 0; i < laser_output_step2_cnt - 1;)
    {
        double x_l, y_l, x_h, y_h, x, y;
        uint16_t index_l = laser_output_step2[i++];
        uint16_t index_h = laser_output_step2[i++];

        x_l =   laser_ranges[index_l][DISTANCE] * cos(laser_ranges[index_l][ANGLE] * angle_step_size + msg->angle_min);
        y_l =   laser_ranges[index_l][DISTANCE] * sin(laser_ranges[index_l][ANGLE] * angle_step_size + msg->angle_min);
        x_h =   laser_ranges[index_h][DISTANCE] * cos(laser_ranges[index_h][ANGLE] * angle_step_size + msg->angle_min);
        y_h =   laser_ranges[index_h][DISTANCE] * sin(laser_ranges[index_h][ANGLE] * angle_step_size + msg->angle_min);

        x = (x_l + x_h) / 2.0;
        y = (y_l + y_h) / 2.0;

        double temp_x = x, temp_y = y;
        get_cord_against_origin(temp_x, temp_y);
        // std::cout << temp_x << " " << temp_y << " " << x << " " << y << " " << frame_id << std::endl;
        if (temp_x < 0 && temp_x > x_limit_against_origin &&
            temp_y < 0 && temp_y > y_limit_against_origin)
        {
            pub_vis(vis_pub, x, y, 0, 1, 0, 0, leg_array_cnt);
        }
        else
        {
            // std::cout << temp_x << " " << temp_y << " " << x << " " << y << " " << frame_id << std::endl;
        }
        
        // std::cout << msg->angle_min  <<std::endl;
        // std::cout << msg->angle_max  <<std::endl;
        // std::cout << tf::getYaw( superior_scan_tf.getRotation()) <<std::endl;
        // std::cout << frame_id << std::endl;

        // laser_touch_screen::PositionMeasurement pos;
        // pos.header.stamp = msg->header.stamp;
        // pos.header.frame_id = frame_id;
        // pos.name = module_namespace;
        // pos.object_id = leg_array_cnt;
        // pos.pos.x = x;
        // pos.pos.y = y;
        // pos.pos.z = 0;
        // pos.reliability = 1;
        // pos.covariance[0] = 10000.0;
        // pos.covariance[1] = 0.0;
        // pos.covariance[2] = 0.0;
        // pos.covariance[3] = 0.0;
        // pos.covariance[4] = 10000.0;
        // pos.covariance[5] = 0.0;
        // pos.covariance[6] = 0.0;
        // pos.covariance[7] = 0.0;
        // pos.covariance[8] = 10000.0;
        // pos.initialization = 0;
        // legs.push_back(pos);

        leg_array_cnt++;
    }
    leg_array.header.stamp = ros::Time::now();
    leg_array.people = legs;
    leg_pub.publish(leg_array);
    leg_array_update = true;
    // printf("leg_array_cnt: %d\r\n", leg_array_cnt);
    // leg_array_cnt = 0;
}

void laser_touch_screen_cls::get_tf()
{
    tf::TransformListener listenner;
    ros::Rate rate(50);

    while (ros::ok())
    {
        try
        {
            listenner.waitForTransform(origin_frame_id, frame_id, ros::Time(0), ros::Duration(60.0));
            origin_scan_tf_mutex.lock();
                listenner.lookupTransform(origin_frame_id, frame_id, ros::Time(0), origin_scan_tf);
            origin_scan_tf_mutex.unlock();
        }
        catch (tf::LookupException &ex)
        {
            ROS_ERROR("Transform error: %s", ex.what());
        }

        // std::cout << origin_scan_tf.getOrigin().x() << std::endl;
        // std::cout << origin_scan_tf.getOrigin().y() << std::endl;
        // std::cout << origin_scan_tf.getOrigin().z() << std::endl;

        // printf ("-=-=-=-=-=-=--=-=-=-=-=-=-=-=\r\n");
        rate.sleep();
    }
}

void laser_touch_screen_cls::pub_tf()
{
    tf::Transform transform;
    ros::Rate rate(50);

    while (ros::ok())
    {
        superior_scan_tf_mutex.lock();
            tf::Quaternion q;
            superior_scan_tf.setOrigin(tf::Vector3(scan_origin_x, scan_origin_y, scan_origin_z));
            q.setRPY(scan_roll, scan_pitch, scan_yaw);
            superior_scan_tf.setRotation(q);

            tf_br.sendTransform(tf::StampedTransform(superior_scan_tf, ros::Time::now(), superior_frame_id, frame_id));
        superior_scan_tf_mutex.unlock();

        rate.sleep();
    }
}

void laser_touch_screen_cls::tf_transform(tf::StampedTransform& transform, double &x, double &y, double &z)
{
    // 解决tf时间戳问题
    tf::Transform mat;
    tf::Vector3 vec_in, vec_out;
    mat.setOrigin(transform.getOrigin());
    mat.setRotation(transform.getRotation());
    vec_in = tf::Vector3(x, y, z);
    vec_out = mat * vec_in;

    x = vec_out.getX();
    y = vec_out.getY();
    z = vec_out.getZ();
}

void laser_touch_screen_cls::get_cord_against_origin(double &x, double &y)
{
    double temp = 0;
    origin_scan_tf_mutex.lock();
        tf_transform(origin_scan_tf, x, y, temp);
    origin_scan_tf_mutex.unlock();;
}