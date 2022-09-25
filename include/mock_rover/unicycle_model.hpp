#ifndef UNICYCLE_MODEL_HPP
#define UNICYCLE_MODEL_HPP
#include <mutex>

#include <boost/math/special_functions/sign.hpp>

#include <Eigen/Dense>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// Shortest time delta; model is updated at 1 / MIN_TIME_STEP_S Hz.
const double MIN_TIME_STEP_S = 0.05;

const double MAX_LINEAR_ACC = 0.5; // m/s/s
const double MAX_DELTA_V = MAX_LINEAR_ACC *  MIN_TIME_STEP_S;
const double MAX_ANGULAR_VEL = 1.0; // radians/s

class VehicleState {
public:
    // Expressed in the odom frame.
    double x{};
    double y{};
    double theta{};

    // Expressed in the vehicle frame.
    double x_dot{};
    double y_dot{};  // Always zero.
    double theta_dot{};
};

class UnicycleModel {
public:
    UnicycleModel() = default;
    UnicycleModel(const UnicycleModel& other) = delete;
    void update_cmd_vel(const geometry_msgs::Twist& cmd_vel)
    {
        std::scoped_lock<std::mutex> lock(cur_cmd_vel_mutex_);
        cur_cmd_vel_ = geometry_msgs::Twist(cmd_vel);
    }

    void update_model()
    {
        std::scoped_lock<std::mutex, std::mutex> lock(vs_mutex_, cur_cmd_vel_mutex_);

        // Update velocities (x, theta). Dont' bother with y; the Unicycle model does
        // not allow movement in the vehicle y-axes.
        double v_diff = cur_cmd_vel_.linear.x - vs_.x_dot;
        if(fabs(v_diff) >= MAX_DELTA_V)
        {
            vs_.x_dot += MAX_DELTA_V * boost::math::sign(v_diff);
        }
        else
        {
            vs_.x_dot = cur_cmd_vel_.linear.x;
        }

        // Update pose.
        vs_.x += vs_.x_dot * MIN_TIME_STEP_S;
    }

    VehicleState get_vehicle_state()
    {
        std::scoped_lock<std::mutex> lock(vs_mutex_);
        return VehicleState{vs_};
    }
private:
    VehicleState vs_;
    std::mutex vs_mutex_;

    geometry_msgs::Twist cur_cmd_vel_;
    std::mutex cur_cmd_vel_mutex_;
};

#endif //UNICYCLE_MODEL_HPP
