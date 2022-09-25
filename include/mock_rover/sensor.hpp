#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <string>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
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

// Construct a nav_msgs::Odometry message from the current vehicle state, and publish it.
void Odometer::update(const ros::TimerEvent& event, const VehicleState& vs) {
    nav_msgs::Odometry odom_msg;
    odom_msg.header.frame_id = "odom";
    odom_msg.child_frame_id = "base_link";

    // Translation.
    odom_msg.pose.pose.position.x = vs.x;
    odom_msg.pose.pose.position.y = vs.y;

    // Heading.
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, vs.theta);
    odom_msg.pose.pose.orientation.w = q.getW();
    odom_msg.pose.pose.orientation.x = q.getX();
    odom_msg.pose.pose.orientation.y = q.getY();
    odom_msg.pose.pose.orientation.z = q.getZ();

    // Velocities.
    odom_msg.twist.twist.linear.x = vs.x_dot;
    odom_msg.twist.twist.angular.z = vs.theta_dot;
    odom_pub_->publish(odom_msg);
}
#endif //SENSOR_HPP
