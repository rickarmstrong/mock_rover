#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <string>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mock_rover/mock_rover.hpp"
#include "mock_rover/unicycle_model.hpp"

class Sensor {
public:
    Sensor() = default;
    virtual ~Sensor() = 0;
    virtual void update(const ros::TimerEvent& event, const VehicleState& vs) = 0;
};
Sensor::~Sensor() = default;

#endif //SENSOR_HPP
