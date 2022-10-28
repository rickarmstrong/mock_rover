#include <functional>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include "mock_rover/mock_rover.hpp"
#include "mock_rover/odometer.hpp"
#include "mock_rover/gps.hpp"

int main(int argc, char** argv){
    ros::init(argc, argv, "mock_rover");
    ros::NodeHandle nh;

    // TODO: validate params.
    float tick_rate;
    ros::param::param<float>("mock_rover/tick_rate_hz", tick_rate, 10.0);

    // Model update timer, ticks all sensors at (1/MIN_TIME_STEP_S) Hz.
    std::unique_ptr<UnicycleModel> um(std::make_unique<UnicycleModel>(tick_rate));
    ros::Timer model_update_timer = nh.createTimer(
            ros::Duration(1.0 / tick_rate),
            boost::bind(update_model_timer_cb, _1, std::ref(um)));

    // Odometer.
    std::string odom_topic;
    float odom_pub_rate;
    ros::param::param<std::string>("mock_rover/odom_topic", odom_topic, "odom");
    ros::param::param<float>("mock_rover/odometer/publish_rate", odom_pub_rate, tick_rate);
    Odometer odometer(odom_topic, odom_pub_rate, um);

    // GPS.
//    float map_origin_lat;
//    float map_origin_lon;
//    float gps_publish_rate;
//    ros::param::get("mock_rover/gps/datum/lat", map_origin_lat);
//    ros::param::get("mock_rover/gps/datum/lon", map_origin_lon);
//    ros::param::param<float>("mock_rover/gps/publish_rate", gps_publish_rate, tick_rate);
//    Datum datum{map_origin_lat, map_origin_lon, 0.0};
//    std::string gps_topic;
//    ros::param::param<std::string>("mock_rover/gps_topic", gps_topic, "gps");
//    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>(gps_topic, 10);
//    auto gps_pub_ptr = std::make_unique<ros::Publisher>(gps_pub);
//    sensors.push_back(std::make_unique<Gps>(gps_pub_ptr, gps_publish_rate, datum));

    // TODO: implement IMU.

    // cmd_vel message subscriber.
    std::string cmd_vel_topic;
    ros::param::param<std::string>("mock_rover/cmd_vel_topic", cmd_vel_topic, "cmd_vel");
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 1, boost::bind(cmd_vel_cb, _1, std::ref(um)));

    // Go.
    ros::spin();
    return(0);
}
