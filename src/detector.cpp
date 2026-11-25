#include <algorithm>
#include <cfloat>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

using namespace ros;
using namespace std;
using namespace sensor_msgs;
using namespace visualization_msgs;

// PUBLISHER
Publisher pub_markers;

// VARIABLES GLOBALES
int NBR_REGION = 8;
int SCAN_SAMPLE = 360;
int ANGLE_REGION = 45;
std::vector<float> min_distances;

// PROTOTYPE DES FONCTIONS
float get_minimum(vector<float> data, int debut, int fin, int *indiceMin);

// FONCTION CALLBACK
// A COMPLETER//
void scanCallback(LaserScan msg) {

  float min_region_i = FLT_MAX;
  min_distances.clear();

  for (int i = 0; i < msg.ranges.size(); i++) {
    if (msg.ranges[i] < msg.range_min || msg.ranges[i] > msg.range_max) {
      msg.ranges[i] = 0.0;
    }
  }

  MarkerArray marker_array;

  for (int i = 0; i < NBR_REGION; i++) {

    int debut = i * ANGLE_REGION;
    int fin = ((i + 1) * ANGLE_REGION) - 1;
    int indice_min = 0;

    min_region_i = get_minimum(msg.ranges, debut, fin, &indice_min);
    min_distances.push_back(min_region_i);

    float angle_milieu_rad =
        (i * ANGLE_REGION + (ANGLE_REGION / 2.0)) * (M_PI / 180.0);
    float angle_obstacle_rad = indice_min * (M_PI / 180.0);

    Marker text_marker;

    text_marker.header.frame_id = "base_scan";
    text_marker.header.stamp = ros::Time::now();
    text_marker.ns = "lidar_points";
    text_marker.id = i;
    text_marker.type = Marker::TEXT_VIEW_FACING;
    text_marker.action = Marker::ADD;

    Marker sphere_marker;
    sphere_marker.header.frame_id = "base_scan";
    sphere_marker.header.stamp = ros::Time::now();
    sphere_marker.ns = "lidar_points";
    sphere_marker.id = i + 10;
    sphere_marker.type = Marker::SPHERE;
    sphere_marker.action = Marker::ADD;

    text_marker.pose.position.x = 1.0 * cos(angle_milieu_rad);
    text_marker.pose.position.y = 1.0 * sin(angle_milieu_rad);
    text_marker.pose.position.z = 0.3;
    text_marker.scale.z = 0.15;
    text_marker.pose.orientation.w = 1.0;
    text_marker.lifetime = Duration(0.5);
    text_marker.color.a = 1.0;

    sphere_marker.pose.position.x = min_region_i * cos(angle_milieu_rad);
    sphere_marker.pose.position.y = min_region_i * sin(angle_milieu_rad);
    sphere_marker.pose.position.z = 0.0;
    sphere_marker.scale.x = 0.2; // 20 cm
    sphere_marker.scale.y = 0.2;
    sphere_marker.scale.z = 0.2;
    sphere_marker.pose.orientation.x = 0.0;
    sphere_marker.pose.orientation.y = 0.0;
    sphere_marker.pose.orientation.z = 0.0;
    sphere_marker.pose.orientation.w = 1.0;
    sphere_marker.lifetime = Duration(0.5);

    if (min_region_i < 0.5) {
      sphere_marker.color.r = 1.0;
      sphere_marker.color.g = 0.0;
      sphere_marker.color.b = 0.0;
      sphere_marker.color.a = 1.0;
    } else {
      text_marker.color.r = 0.0;
      text_marker.color.g = 1.0;
      text_marker.color.b = 0.0;
      text_marker.text = "safe";
    }
    text_marker.color.a = 1.0;

    marker_array.markers.push_back(text_marker);
    marker_array.markers.push_back(sphere_marker);
  }

  pub_markers.publish(marker_array);
}

int main(int argc, char **argv) {
  init(argc, argv, "detector");
  NodeHandle nh;

  Rate loop_rate(10);

  Subscriber odom_sub = nh.subscribe("/scan", 100, scanCallback);
  pub_markers = nh.advertise<MarkerArray>("/lidar_visualization", 10);
  spin();

  return 0;
}

float get_minimum(vector<float> data, int debut, int fin, int *indice_min) {

  float min = FLT_MAX;

  for (int i = debut; i <= fin; i++) {
    if (data[i] < min && data[i] > 0.0) {
      min = data[i];
      *indice_min = i;
    }
  }
  return std::min(10.0f, min);
}