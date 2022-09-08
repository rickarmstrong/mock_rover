#ifndef DEAD_SIMPLE_SIM_HPP
#define DEAD_SIMPLE_SIM_HPP
#include <mutex>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Shortest time delta; model is updated at 1 / MIN_TIME_STEP_S.
const double MIN_TIME_STEP_S = 0.1;

// Vehicle dymamics.
const double MAX_LINEAR_ACC = 0.5; // m/s/s
const double MAX_ANGULAR_VEL = 1.0; // radians/s

class VehicleState {
public:
    double x{};
    double y{};
    double theta{};
    double x_dot{};
    double y_dot{};
    double theta_dot{};
};

class UnicycleModel {
public:
    UnicycleModel() = default;
    UnicycleModel(const UnicycleModel& other) = delete;
    void update_cmd_vel(const geometry_msgs::Twist& cmd_vel)
    {
        std::scoped_lock<std::mutex> lock(cur_cmd_vel_mutex);
        cur_cmd_vel = geometry_msgs::Twist(cmd_vel);
    }

    void update_model()
    {
        std::scoped_lock<std::mutex, std::mutex> lock(vs_mutex, cur_cmd_vel_mutex);
        vs.x_dot += (MAX_LINEAR_ACC * MIN_TIME_STEP_S);
    }

    VehicleState get_vehicle_state()
    {
        std::scoped_lock<std::mutex> lock(vs_mutex);
        return VehicleState{vs};
    }
private:
    VehicleState vs;
    std::mutex vs_mutex;

    geometry_msgs::Twist cur_cmd_vel;
    std::mutex cur_cmd_vel_mutex;
};

void update_model_timer_cb(const ros::TimerEvent& event, UnicycleModel* um)
{
    um->update_model();
}

void cmd_vel_cb(const geometry_msgs::TwistConstPtr& cmd_vel, UnicycleModel* um)
{
    um->update_cmd_vel(*cmd_vel);
}
#endif // DEAD_SIMPLE_SIM_HPP
