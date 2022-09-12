#include <functional>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <mock_rover/mock_rover.hpp>
#include <mock_rover/sensor.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "mock_rover");
    ros::NodeHandle nh;

    // Sensors receive all vehicle state updates, and publish on their own respective schedules.
    std::vector<std::unique_ptr<Sensor>> sensors;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("/odom", 10);
    auto odom_pub_ptr = std::make_unique<ros::Publisher>(odom_pub);
    sensors.push_back(std::make_unique<Odometer>(odom_pub_ptr));

    // Model update timer, ticks all sensors at (1/MIN_TIME_STEP_S) Hz.
    UnicycleModel um;
    ros::Timer model_update_timer = nh.createTimer(
            ros::Duration(MIN_TIME_STEP_S),
            boost::bind(update_model_timer_cb, _1, &um, std::ref(sensors)));

    // cmd_vel message subscriber.
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(cmd_vel_cb,_1, &um));

    // Go.
    ros::spin();
    return(0);
}
