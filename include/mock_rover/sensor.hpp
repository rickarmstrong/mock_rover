#ifndef SENSOR_HPP
#define SENSOR_HPP
#include <string>
#include <utility>

#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "mock_rover/unicycle_model.hpp"

/** \class Sensor
 *
 * Abstract base class for a sensor model that takes the instantaneous vehicle state,
 * transforms the data in some way, and publishes on a single ROS topic. Publishing occurs
 * on a schedule set by subclasses, using a constructor template supplied by Sensor.
 * We assume that every subclass publishes to _one_ topic.
 */
class Sensor {
public:
    Sensor() = delete;
    virtual ~Sensor() = 0;

    /**
     * Timer callback to be implemented by subclasses.
     * @param event Timing info from the ros::Timer that invoked the callback.
     */
    virtual void update(const ros::TimerEvent& event) = 0;

protected:
    /**
     * Constructor template that lets subclasses pass in type information for for use by ROS function templates.
     * @tparam T Deduced from the 'self' parameter (i.e. the subclass's 'this' pointer).
     * @param topic ROS topic associated with the sensor. The subclass must declare the ROS `msg_type` in a
     *  public typedef, e.g.: `typedef foo_msgs::FooBar msg_type`.
     * @param publish_rate Rate at which the sensor outputs messages on the associated ROS topic.
     * @param um The vehicle model, queried at each update cycle.
     * @param self `this` pointer of the calling instance.
     */
    template<typename T>
    explicit Sensor(std::string topic, double publish_rate, std::unique_ptr<UnicycleModel>& um, T* self)
            : output_topic_(std::move(topic)), pub_rate_(publish_rate), um_(um) {
        ros::NodeHandle nh;
        publisher_ = nh.advertise<typename T::msg_type>(output_topic_, T::PUB_QUEUE_SIZE);
        timer_ = nh.createTimer<T>(ros::Duration(1.0 / pub_rate_), &T::update, self);
    }

    const std::string output_topic_;
    const double pub_rate_;
    ros::Publisher publisher_;
    ros::Timer timer_;
    std::unique_ptr<UnicycleModel>& um_;
};
Sensor::~Sensor() = default;

#endif //SENSOR_HPP
