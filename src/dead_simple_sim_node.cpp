#include <mutex>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <dead_simple_sim/dead_simple_sim.hpp>

void update_model_timer_cb(const ros::TimerEvent& event, UnicycleModel& um)
{
    um.update_model();
}

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& cmd_vel, UnicycleModel& um)
{
    um.update_cmd_vel(*cmd_vel);
}

int main(int argc, char** argv){
    ros::init(argc, argv, "dead_simple_sim");
    ros::NodeHandle nh;

    UnicycleModel um;

    // Model update.
    ros::Timer model_update_timer = nh.createTimer(
            ros::Duration(MIN_TIME_STEP_S), boost::bind(update_model_timer_cb, _1, um));

    // cmd_vel message subscriber.
    ros::Subscriber cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, boost::bind(cmd_vel_cb,_1, um));

    ros::spin();
    return(0);
}
