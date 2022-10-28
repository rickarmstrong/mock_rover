#ifndef GPS_HPP
#define GPS_HPP
#include <GeographicLib/GeoCoords.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "mock_rover/sensor.hpp"

// Geodetic origin of the map frame.
typedef struct DatumTag {
    double lat;
    double lon;
    double heading_ned_degrees;
} Datum;

class Gps : public Sensor {
public:
    ~Gps() override = default;
    Gps() = delete;
    Gps(const Gps& g) = delete;
    explicit Gps(const std::string& topic, double publish_rate, std::unique_ptr<UnicycleModel>& um, Datum datum);
    void update(const ros::TimerEvent&) override;
    [[nodiscard]] GeographicLib::GeoCoords odom_to_lat_lon(double x, double y) const;
private:
    Datum datum_;
};

Gps::Gps(const std::string& topic, double publish_rate, std::unique_ptr<UnicycleModel> &um, Datum datum)
        :Sensor(topic, publish_rate, um), datum_(datum)
{
    // TODO: push this up to Sensor.
    ros::NodeHandle nh;
    publisher_ = nh.advertise<sensor_msgs::NavSatFix>(topic, PUB_QUEUE_SIZE);
    timer_ = nh.createTimer(ros::Duration(1.0 / pub_rate_), &Gps::update, this);
}

// Construct a nav_msgs::NavSatFix message from the current vehicle position, and publish it.
void Gps::update(const ros::TimerEvent& event)
{
    VehicleState vs = um_->get_vehicle_state();

    // TODO: populate header and covariances.
    GeographicLib::GeoCoords xy = odom_to_lat_lon(vs.x, vs.y);
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = xy.Latitude();
    gps_msg.longitude = xy.Longitude();
    publisher_.publish(gps_msg);
}

GeographicLib::GeoCoords Gps::odom_to_lat_lon(double x, double y) const
{
    // TODO: account for the map->odom transform.
    GeographicLib::GeoCoords map_origin{datum_.lat, datum_.lon};
    GeographicLib::GeoCoords ll{
        map_origin.Zone(),
        map_origin.Northp(),
        map_origin.Easting() + x,
        map_origin.Northing() + y};
    return ll;
}
#endif //GPS_HPP
