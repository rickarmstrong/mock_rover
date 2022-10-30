#ifndef ODOMETER_HPP
#define ODOMETER_HPP
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

#include <utility>

#include "mock_rover/sensor.hpp"

constexpr int PUB_QUEUE_SIZE = 10;

class Odometer : public Sensor {
public:
    ~Odometer() override = default;
    Odometer() = delete;
    Odometer(const Odometer& o) = delete;
    explicit Odometer(const std::string& topic, double publish_rate, std::unique_ptr<UnicycleModel>& um)
            :Sensor(topic, publish_rate, um, this) {}
    void update(const ros::TimerEvent& event) override;

    // Publisher params.
    static constexpr int PUB_QUEUE_SIZE = 10;
    typedef nav_msgs::Odometry msg_type;
};

// Construct a nav_msgs::Odometry message from the current vehicle state, and publish it.
void Odometer::update(const ros::TimerEvent& event) {

    VehicleState vs = um_->get_vehicle_state();

    // TODO: populate header and covariances.
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
    publisher_.publish(odom_msg);
}
#endif //ODOMETER_HPP
