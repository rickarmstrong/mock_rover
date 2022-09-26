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

    // Sensors receive all vehicle state updates, and publish on their own respective schedules.
    // main() creates and owns the ROS publishers.
    std::vector<std::unique_ptr<Sensor>> sensors;

    // Odometer.
    std::string odom_topic;
    ros::param::param<std::string>("mock_rover/odom_topic", odom_topic, "odom");
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 10);
    auto odom_pub_ptr = std::make_unique<ros::Publisher>(odom_pub);
    sensors.push_back(std::make_unique<Odometer>(odom_pub_ptr));

    // GPS.
    float map_origin_lat;
    float map_origin_lon;
    ros::param::get("mock_rover/datum/lat", map_origin_lat);
    ros::param::get("mock_rover/datum/lon", map_origin_lon);
    Datum datum{map_origin_lat, map_origin_lon, 0.0};
    std::string gps_topic;
    ros::param::param<std::string>("mock_rover/gps_topic", gps_topic, "gps");
    ros::Publisher gps_pub = nh.advertise<sensor_msgs::NavSatFix>(gps_topic, 10);
    auto gps_pub_ptr = std::make_unique<ros::Publisher>(gps_pub);
    sensors.push_back(std::make_unique<Gps>(gps_pub_ptr, datum));

    // TODO: implement IMU.

    // Model update timer, ticks all sensors at (1/MIN_TIME_STEP_S) Hz.
    float tick_rate;
    ros::param::param<float>("mock_rover/tick_rate_hz", tick_rate, 10.0);
    UnicycleModel um(tick_rate);
    ros::Timer model_update_timer = nh.createTimer(
            ros::Duration(1.0 / tick_rate),
            boost::bind(update_model_timer_cb, _1, &um, std::ref(sensors)));

    // cmd_vel message subscriber.
    std::string cmd_vel_topic;
    ros::param::param<std::string>("mock_rover/cmd_vel_topic", cmd_vel_topic, "cmd_vel");
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(cmd_vel_topic, 1, boost::bind(cmd_vel_cb,_1, &um));

    // Go.
    ros::spin();
    return(0);
}
