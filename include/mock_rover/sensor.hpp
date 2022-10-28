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
    explicit Sensor(std::string topic, double publish_rate, std::unique_ptr<UnicycleModel>& um)
        : output_topic_(std::move(topic)), pub_rate_(publish_rate), um_(um) {}
    virtual ~Sensor() = 0;
    virtual void update(const ros::TimerEvent& event) = 0;

    // Initialize our publisher and timer.
    template <typename T>
    void init(T* self){
        ros::NodeHandle nh;
        publisher_ = nh.advertise<typename T::msg_type>(output_topic_, T::PUB_QUEUE_SIZE);
        timer_ = nh.createTimer<T>(ros::Duration(1.0 / pub_rate_), &T::update, self);
    }
protected:
        const std::string output_topic_;
        const double pub_rate_;
        ros::Publisher publisher_;
        ros::Timer timer_;
        std::unique_ptr<UnicycleModel>& um_;
};
Sensor::~Sensor() = default;

#endif //SENSOR_HPP
