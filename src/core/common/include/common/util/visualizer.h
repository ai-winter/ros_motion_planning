/**
 * *********************************************************
 *
 * @file: visualizer.h
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
#ifndef RMP_COMMON_UTIL_VISUALIZER_H_
#define RMP_COMMON_UTIL_VISUALIZER_H_

#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <tf2/LinearMath/Quaternion.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/ColorRGBA.h>
#include <visualization_msgs/Marker.h>

#include "common/geometry/point.h"
#include "common/structure/singleton.h"

namespace rmp {
namespace common {
namespace util {
class Visualizer {
public:
  using Line2d = std::pair<common::geometry::Point2d, common::geometry::Point2d>;
  using Lines2d = std::vector<Line2d>;

public:
  /**
   * @brief Publish path
   * @param path planning path
   * @param publisher publisher
   */
  template <typename Point>
  static void publishPlan(const std::vector<Point>& plan, const ros::Publisher& publisher,
                          const std::string& frame_id) {
    // create visulized path plan
    nav_msgs::Path gui_plan;
    gui_plan.poses.resize(plan.size());
    gui_plan.header.frame_id = frame_id;
    gui_plan.header.stamp = ros::Time::now();
    for (unsigned int i = 0; i < plan.size(); i++) {
      tf2::Quaternion q;
      q.setRPY(0.0, 0.0, plan[i].theta());
      gui_plan.poses[i].header.stamp = ros::Time::now();
      gui_plan.poses[i].header.frame_id = frame_id;
      gui_plan.poses[i].pose.position.x = plan[i].x();
      gui_plan.poses[i].pose.position.y = plan[i].y();
      gui_plan.poses[i].pose.position.z = 0.0;
      gui_plan.poses[i].pose.orientation.x = q.getX();
      gui_plan.poses[i].pose.orientation.y = q.getY();
      gui_plan.poses[i].pose.orientation.z = q.getZ();
      gui_plan.poses[i].pose.orientation.w = q.getW();
    }

    publisher.publish(gui_plan);
  }

  template <typename Point>
  static void publishExpandZone(const std::vector<Point>& expand,
                                const costmap_2d::Costmap2D* costmap,
                                const ros::Publisher& publisher,
                                const std::string& frame_id) {
    nav_msgs::OccupancyGrid grid;
    float resolution = costmap->getResolution();

    // build expand
    grid.header.frame_id = frame_id;
    grid.header.stamp = ros::Time::now();
    grid.info.resolution = resolution;
    grid.info.width = costmap->getSizeInCellsX();
    grid.info.height = costmap->getSizeInCellsY();

    double wx, wy;
    costmap->mapToWorld(0, 0, wx, wy);
    grid.info.origin.position.x = wx - resolution / 2;
    grid.info.origin.position.y = wy - resolution / 2;
    grid.info.origin.position.z = 0.0;
    grid.info.origin.orientation.w = 1.0;
    grid.data.resize(grid.info.width * grid.info.height);

    for (unsigned int i = 0; i < grid.data.size(); i++) {
      grid.data[i] = 0;
    }

    auto grid2index = [&](const Point& pt) {
      return static_cast<int>(pt.x()) +
             static_cast<int>(grid.info.width) * static_cast<int>(pt.y());
    };

    for (const auto& pt : expand) {
      grid.data[grid2index(pt)] = 50;
    }

    publisher.publish(grid);
  }

  /**
   * @brief publish points
   */
  template <typename Point>
  static void publishPoints(const std::vector<Point>& points,
                            const ros::Publisher& publisher, const std::string& frame_id,
                            const std::string& ns, std_msgs::ColorRGBA color = RED,
                            double scale = 0.2, int type = SPHERE) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = 0;

    switch (type) {
      case CUBE:
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        break;
      case SPHERE:
      default:
        marker.type = visualization_msgs::Marker::SPHERE_LIST;
    }

    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;

    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color = color;

    marker.points.reserve(points.size());
    for (const auto& pt : points) {
      geometry_msgs::Point p;
      p.x = pt.x();
      p.y = pt.y();
      p.z = 0.0;
      marker.points.push_back(p);
    }

    if (!marker.points.empty()) {
      publisher.publish(marker);
    }
  }

  /**
   * @brief publish lines
   */
  static void publishLines2d(const Lines2d& lines, const ros::Publisher& publisher,
                             const std::string& frame_id, const std::string& ns,
                             std_msgs::ColorRGBA color = RED, double scale = 0.2);

private:
  static std_msgs::ColorRGBA _colorInit(double r, double g, double b, double a);

public:
  static std_msgs::ColorRGBA RED;
  static std_msgs::ColorRGBA DARK_GREEN;
  static std_msgs::ColorRGBA PURPLE;
  static std_msgs::ColorRGBA LIGHT_PURPLE;
  static std_msgs::ColorRGBA LIGHT_ORANGE;
  enum MARKER_TYPE {
    CUBE = 0,
    SPHERE = 1,
  };
};

using VisualizerPtr = rmp::common::structure::Singleton<Visualizer>;

}  // namespace util
}  // namespace common
}  // namespace rmp

#endif