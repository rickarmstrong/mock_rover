#ifndef DEAD_SIMPLE_SIM_HPP
#define DEAD_SIMPLE_SIM_HPP

// Shortest time delta; model is updated at 1 / MIN_TIME_STEP_S.
const double MIN_TIME_STEP_S = 1.0;

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
        std::lock_guard<std::mutex> lock(this->cur_cmd_vel_mutex);
        cur_cmd_vel = geometry_msgs::Twist(cmd_vel);
    }

    void update_model()
    {
        std::lock_guard<std::mutex> lock(this->vs_mutex);
    }

private:
    VehicleState vs;
    std::mutex vs_mutex;

    geometry_msgs::Twist cur_cmd_vel;
    std::mutex cur_cmd_vel_mutex;
};

#endif // DEAD_SIMPLE_SIM_HPP
