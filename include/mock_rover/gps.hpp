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
    explicit Gps(const std::unique_ptr<ros::Publisher>& pub, Datum datum) : Sensor(pub), datum_(datum) {}
    void update(const ros::TimerEvent&, const VehicleState& vs) override;
    [[nodiscard]] GeographicLib::GeoCoords odom_to_lat_lon(double x, double y) const;
private:
    Datum datum_;
};

// Construct a nav_msgs::NavSatFix message from the current vehicle position, and publish it.
void Gps::update(const ros::TimerEvent& event, const VehicleState& vs)
{
    // TODO: populate header and covariances.
    GeographicLib::GeoCoords xy = odom_to_lat_lon(vs.x, vs.y);
    sensor_msgs::NavSatFix gps_msg;
    gps_msg.latitude = xy.Latitude();
    gps_msg.longitude = xy.Longitude();
    pub_->publish(gps_msg);
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
