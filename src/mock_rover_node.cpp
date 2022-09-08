#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <mock_rover/mock_rover.hpp>

int main(int argc, char** argv){
    ros::init(argc, argv, "mock_rover");
    ros::NodeHandle nh;

    UnicycleModel um;

    // Model update, at (1/MIN_TIME_STEP_S) Hz.
    ros::Timer model_update_timer = nh.createTimer(
            ros::Duration(MIN_TIME_STEP_S), boost::bind(update_model_timer_cb, _1, &um));

    // cmd_vel message subscriber.
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(cmd_vel_cb,_1, &um));

    ros::spin();
    return(0);
}
