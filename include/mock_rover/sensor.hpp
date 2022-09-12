#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <string>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <mock_rover/mock_rover.hpp>
#include <mock_rover/unicycle_model.hpp>

class Sensor {
public:
    Sensor() = default;
    virtual ~Sensor() = 0;
    virtual void update(const ros::TimerEvent& event, const VehicleState& vs) = 0;
};
Sensor::~Sensor() = default;

class Odometer : public Sensor {
    static constexpr double PUBLISH_RATE_HZ = 3.0;
public:
    ~Odometer() override = default;
    Odometer() = delete;
    Odometer(const Odometer& o) = delete;
    explicit Odometer(const std::unique_ptr<ros::Publisher>& pub) : odom_pub_(pub) {}
    void update(const ros::TimerEvent& event, const VehicleState& vs) override;

private:
    const std::unique_ptr<ros::Publisher>& odom_pub_;
};

void Odometer::update(const ros::TimerEvent& event, const VehicleState& vs) {
    nav_msgs::Odometry odom_msg;
    odom_msg.twist.twist.linear.x = vs.x_dot;
    odom_msg.pose.pose.position.x = vs.x;
    odom_pub_->publish(odom_msg);
}
#endif //SENSOR_HPP
