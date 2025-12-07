/**
 * *********************************************************
 *
 * @file: visualizer.cpp
 * @brief: Contains visualization functions
 * @author: Yang Haodong
 * @date: 2024-09-24
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <visualization_msgs/MarkerArray.h>

#include "common/util/visualizer.h"

namespace rmp {
namespace common {
namespace util {
std_msgs::ColorRGBA Visualizer::RED = Visualizer::_colorInit(1.0, 0.0, 0.0, 1.0);
std_msgs::ColorRGBA Visualizer::DARK_GREEN =
    Visualizer::_colorInit(0.43, 0.54, 0.24, 0.5);
std_msgs::ColorRGBA Visualizer::PURPLE = Visualizer::_colorInit(1.0, 0.0, 1.0, 1.0);

/**
 * @brief publish lines
 */
void Visualizer::publishLines2d(const Lines2d& lines, const ros::Publisher& publisher,
                                const std::string& frame_id, const std::string& ns,
                                std_msgs::ColorRGBA color, double scale) {
  visualization_msgs::Marker line_marker;
  line_marker.header.frame_id = frame_id;
  line_marker.header.stamp = ros::Time::now();
  line_marker.ns = ns;
  line_marker.id = 0;
  line_marker.type = visualization_msgs::Marker::LINE_LIST;
  line_marker.action = visualization_msgs::Marker::ADD;
  line_marker.lifetime = ros::Duration(0.5);
  line_marker.pose.orientation.w = 1.0;
  line_marker.scale.x = scale;
  line_marker.scale.y = scale;
  line_marker.scale.z = scale;
  line_marker.color = color;

  for (const auto& line : lines) {
    geometry_msgs::Point start, end;

    start.x = line.first.x();
    start.y = line.first.y();
    start.z = 0.0;

    end.x = line.second.x();
    end.y = line.second.y();
    end.z = 0.0;

    line_marker.points.push_back(start);
    line_marker.points.push_back(end);
  }

  if (!line_marker.points.empty()) {
    publisher.publish(line_marker);
  }
}

std_msgs::ColorRGBA Visualizer::_colorInit(double r, double g, double b, double a) {
  std_msgs::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

}  // namespace util
}  // namespace common
}  // namespace rmp