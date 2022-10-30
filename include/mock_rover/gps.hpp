#ifndef GPS_HPP
#define GPS_HPP
#include <GeographicLib/GeoCoords.hpp>

#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#include "mock_rover/sensor.hpp"

// Geodetic origin and heading of the map frame.
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
    explicit Gps(const std::string& topic, double publish_rate, std::unique_ptr<UnicycleModel>& um, Datum datum)
            :Sensor(topic, publish_rate, um, this), datum_(datum) {};
    void update(const ros::TimerEvent&) override;
    [[nodiscard]] GeographicLib::GeoCoords odom_to_lat_lon(double x, double y) const;

    // Publisher params.
    static constexpr int PUB_QUEUE_SIZE = 10;
    typedef sensor_msgs::NavSatFix msg_type;

private:
    Datum datum_;
};

/**
 * Timer callback that constructs a nav_msgs::NavSatFix message from the current vehicle position, and publishes it.
 * @param event Timing info from the ros::Timer that invoked the callback.
 */
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

/**
 * Transform a pair of x-y coordinates, expressed in the odometry frame, into lat/lon.
 */
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
