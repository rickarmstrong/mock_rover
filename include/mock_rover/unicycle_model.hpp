#ifndef UNICYCLE_MODEL_HPP
#define UNICYCLE_MODEL_HPP
#include <mutex>

#include <boost/math/special_functions/sign.hpp>

#include <Eigen/Dense>

#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

static constexpr double MAX_LINEAR_ACC = 0.5; // m/s^2
static constexpr double MAX_ANGULAR_VEL = 1.0; // radians/s

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
    explicit UnicycleModel(double tick_rate):
        tick_rate_(tick_rate),
        max_delta_v(MAX_LINEAR_ACC * tick_rate),
        max_delta_theta(MAX_ANGULAR_VEL * tick_rate){}
    UnicycleModel() = delete;
    UnicycleModel(const UnicycleModel &other) = delete;

    void update_cmd_vel(const geometry_msgs::Twist &cmd_vel) {
        std::scoped_lock<std::mutex> lock(cur_cmd_vel_mutex_);
        cur_cmd_vel_ = geometry_msgs::Twist(cmd_vel);
    }

    void update_model() {
        std::scoped_lock<std::mutex, std::mutex> lock(vs_mutex_, cur_cmd_vel_mutex_);

        // Increment the model state one time step.
        incr_linear_vel();
        incr_angular_vel();
        incr_pose();
    }

    VehicleState get_vehicle_state() {
        std::scoped_lock<std::mutex> lock(vs_mutex_);
        return VehicleState{vs_};
    }

    double get_tick_rate() const { return tick_rate_; }

private:
    double tick_rate_;

    // Maximum linear velocity increment per model update cycle.
    double max_delta_v;

    // Maximum angular velocity increment per model update cycle.
    double max_delta_theta;

    VehicleState vs_;
    std::mutex vs_mutex_;

    // Current commanded velocity (linear, angular).
    geometry_msgs::Twist cur_cmd_vel_;
    std::mutex cur_cmd_vel_mutex_;

    double incr_linear_vel()
    {
        // Update velocity. Don't bother with y; the Unicycle model does
        // not allow movement in the vehicle y-axes.
        double v_diff = cur_cmd_vel_.linear.x - vs_.x_dot; // Difference between commanded and current linear velocity.
        if (fabs(v_diff) >= max_delta_v) {
            vs_.x_dot += max_delta_v * boost::math::sign(v_diff);
        } else {
            vs_.x_dot = cur_cmd_vel_.linear.x;
        }
    }

    double incr_angular_vel()
    {
        // theta_diff: difference between commanded and current angular velocity.
        double theta_diff = cur_cmd_vel_.angular.z - vs_.theta_dot;
        if (fabs(theta_diff) >= max_delta_theta) {
            vs_.theta_dot += max_delta_theta * boost::math::sign(theta_diff);
        } else {
            vs_.theta_dot = cur_cmd_vel_.angular.z;
        }
    }

    double incr_pose()
    {
        vs_.theta += vs_.theta_dot * (1.0 / tick_rate_);
        vs_.x += vs_.x_dot * std::cos(vs_.theta) * (1.0 / tick_rate_);
        vs_.y += vs_.x_dot * std::sin(vs_.theta) * (1.0 / tick_rate_);
    }
};

#endif //UNICYCLE_MODEL_HPP
