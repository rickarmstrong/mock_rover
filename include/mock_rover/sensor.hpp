#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <string>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mock_rover/unicycle_model.hpp"

/**
 * Base class for a sensor model that takes the instantaneous vehicle state,
 * transforms the data in some way, and publishes on a single ROS topic.
 * We assume that every subclass publishes to _one_ topic.
 */
class Sensor {
public:
    Sensor() = delete;
    explicit Sensor(const std::unique_ptr<ros::Publisher>& pub) : pub_(pub) {}
    virtual ~Sensor() = 0;
    virtual void update(const ros::TimerEvent& event, const VehicleState& vs) = 0;

protected:
        const std::unique_ptr<ros::Publisher>& pub_;
};
Sensor::~Sensor() = default;

#endif //SENSOR_HPP
