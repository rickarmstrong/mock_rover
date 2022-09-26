#ifndef ODOMETER_HPP
#define ODOMETER_HPP
#include "mock_rover/sensor.hpp"

class Odometer : public Sensor {
    // TODO: allow for differing publishing rates.
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
#endif //ODOMETER_HPP
