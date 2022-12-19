#ifndef IMU_HPP
#define IMU_HPP
#include <cmath>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include "mock_rover/sensor.hpp"

class Imu : public Sensor {
public:
    ~Imu() override = default;
    Imu() = delete;
    Imu(const Imu& i) = delete;
    explicit Imu(const std::string& topic, double publish_rate, std::unique_ptr<UnicycleModel>& um)
        :Sensor(topic, publish_rate, um, this) {};
    void update(const ros::TimerEvent&) override;

    // Publisher params.
    static constexpr int PUB_QUEUE_SIZE = 10;
    typedef sensor_msgs::Imu msg_type;
};

/**
 * Timer callback that constructs a sensor_msgs::Imu message from the successive vehicle states, and publishes it.
 * @param event Timing info from the ros::Timer that invoked the callback.
 */
void Imu::update(const ros::TimerEvent &event)
{
    static VehicleState prev_state;
    VehicleState cur_state = um_->get_vehicle_state();

    // Populate the new message. Assume things about the (unicycle) model, namely
    // that only x-axis motion and z-axis rotation are possible.
    sensor_msgs::Imu msg;
    double delta_t = event.current_real.toSec() - event.last_real.toSec();
    msg.linear_acceleration.x = (cur_state.x_dot - prev_state.x_dot) / delta_t;
    msg.linear_acceleration.y = 0;
    msg.linear_acceleration.z = 0;
    msg.angular_velocity.x = 0;
    msg.angular_velocity.y = 0;
    msg.angular_velocity.z = cur_state.theta_dot;
    msg.orientation.w = cos(cur_state.theta / 2.0);
    msg.orientation.x = 0.0;
    msg.orientation.y = 0.0;
    msg.orientation.z = sin(cur_state.theta / 2.0);

    // Send it.
    prev_state = cur_state;
    publisher_.publish(msg);
}
#endif // IMU_HPP
