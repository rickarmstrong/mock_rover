#ifndef MOCK_ROVER_HPP
#define MOCK_ROVER_HPP

#include <mock_rover/sensor.hpp>
#include <mock_rover/unicycle_model.hpp>

void update_model_timer_cb(const ros::TimerEvent& event,
                           UnicycleModel* um,
                           const std::vector<std::unique_ptr<Sensor>>& sensors)
{
    // Tick the model.
    um->update_model();

    // Pass the update along to all sensors.
    VehicleState vs = um->get_vehicle_state();
    for(auto & sensor : sensors) {
        sensor->update(event, vs);
    }
}

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& cmd_vel, UnicycleModel* um)
{
    um->update_cmd_vel(*cmd_vel);
}
#endif // MOCK_ROVER_HPP
