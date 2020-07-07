//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    gps_translator.cc
\brief   Translate from GPS to map coordinates
\author  Joydeep Biswas, (C) 2020
\author   Maxwell Svetlik
*/
//========================================================================

#include <stdio.h>

#include <cmath>
#include <string>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "geometry_msgs/PointStamped.h"
#include "helpers.h"
#include "math_util.h"

using Eigen::Affine2d;
using Eigen::Rotation2Dd;
using Eigen::Vector2d;
using sensor_msgs::NavSatFix;
using std::string;

//DEFINE_string(map, "UT_Campus", "Map name to load");
//DEFINE_string(gps_topic, "gps/fix", "ROS topic for GPS messages");
//DEFINE_string(maps_dir, "enml/maps", "Maps directory");

std::string map;
std::string maps_dir;
std::string gps_topic;

string GetMapFileFromName(const string& map) {
  return maps_dir + "/" + map + "/" + map + ".vectormap.txt";
}

struct GPSToMetric {
  bool Load(const std::string& map) {
    const string file = GetMapFileFromName(map);
    ROS_INFO("Opening file: %s", file.c_str());
    ScopedFile fid(file, "r", true);
    if (fid() == nullptr) return false;
    if (fscanf(fid(), "%lf, %lf, %lf", &gps_origin_latitude, 
        &gps_origin_longitude, &map_orientation) != 3) {
      return false;
    }
    ROS_INFO("Map origin: %12.8lf, %12.8lf\n", gps_origin_latitude, gps_origin_longitude);
    return true;
  }

  Vector2d GpsToMetric(const double latitude, const double longitude) {
    const double theta = math_util::DegToRad(latitude);
    const double c = std::cos(theta);
    const double s = std::sin(theta);
    const double r = sqrt(math_util::Sq(wgs_84_a * wgs_84_b) / (math_util::Sq(c * wgs_84_b) + math_util::Sq(s * wgs_84_a)));
    const double dlat = math_util::DegToRad(latitude - gps_origin_latitude);
    const double dlong = math_util::DegToRad(longitude - gps_origin_longitude);
    const double r1 = r * c;
    const double x = r1 * dlong;
    const double y = r * dlat;
    return Rotation2Dd(map_orientation) * Vector2d(x, y);
  }

  double gps_origin_longitude;
  double gps_origin_latitude;
  double map_orientation;
  
  // Earth geoid parameters from WGS 84 system
  // https://en.wikipedia.org/wiki/World_Geodetic_System#A_new_World_Geodetic_System:_WGS_84
  // a = Semimajor (Equatorial) axis
  static constexpr double wgs_84_a = 6378137.0;
  // b = Semiminor (Polar) axis
  static constexpr double wgs_84_b = 6356752.314245;
};

GPSToMetric map_;
ros::Publisher localization_pub_;
geometry_msgs::PointStamped localization_msg_;

void GpsCallback(const NavSatFix& msg) {
  const bool verbose = true;
  if (verbose) {
    printf("Status:%d Service:%d Lat,Long:%12.8lf, %12.8lf Alt:%7.2lf", 
          msg.status.status, msg.status.service,
          msg.latitude, msg.longitude, msg.altitude);
  }
  if (msg.status.status == msg.status.STATUS_NO_FIX) {
    if (verbose) printf("\n");
    return;
  }
  const Vector2d p = map_.GpsToMetric(msg.latitude, msg.longitude);
  if (verbose) printf(" X,Y: %9.3lf,%9.3lf\n", p.x(), p.y());
  
  localization_msg_.point.x = p.x();
  localization_msg_.point.y = p.y();
  localization_pub_.publish(localization_msg_);
}

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "smads_gps_translator");
  ros::NodeHandle n("~");
  n.getParam("map_name", map);
  n.getParam("maps_dir", maps_dir);
  n.getParam("gps_topic", gps_topic);
  ros::Subscriber gps_sub = n.subscribe(gps_topic, 1, &GpsCallback);
  localization_msg_.header.frame_id = "map";
  localization_msg_.header.seq = 0;
  localization_msg_.header.stamp = ros::Time::now();
  
  localization_pub_ = 
      n.advertise<geometry_msgs::PointStamped>("gps_localization", 1);
  map_.Load(map);
  ros::spin();
  return 0;
}
