#ifndef DEAD_SIMPLE_SIM_HPP
#define DEAD_SIMPLE_SIM_HPP

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
    UnicycleModel(): vs(VehicleState()) {};
    void update_model(){}
    void update_cmd_vel(const geometry_msgs::Twist& cmd_vel)
    {
        this->cur_cmd_vel = geometry_msgs::Twist(cmd_vel);
    }

private:
    VehicleState vs;
    geometry_msgs::Twist cur_cmd_vel;
};

// Shortest time delta; model is updated at this rate.
const double MIN_TIME_STEP_S = 1.0;

#endif // DEAD_SIMPLE_SIM_HPP
