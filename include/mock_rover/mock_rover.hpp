#ifndef MOCK_ROVER_HPP
#define MOCK_ROVER_HPP

#include "mock_rover/sensor.hpp"
#include "mock_rover/unicycle_model.hpp"

void update_model_timer_cb(const ros::TimerEvent& event, std::unique_ptr<UnicycleModel>& um)
{
    // Tick the model.
    um->update_model();
}

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& cmd_vel, std::unique_ptr<UnicycleModel>& um)
{
    um->update_cmd_vel(*cmd_vel);
}
#endif // MOCK_ROVER_HPP
