/**
 * *********************************************************
 *
 * @file: math_helper.cpp
 * @brief: Contains common/commonly used math function
 * @author: Yang Haodong
 * @date: 2024-01-03
 * @version: 1.2
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "math_helper.h"

namespace helper
{
/**
 * @brief Calculate distance between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return distance between nodes
 */
double dist(const Node& node1, const Node& node2)
{
  return std::hypot(node1.x_ - node2.x_, node1.y_ - node2.y_);
}

double dist(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return std::hypot(node1.first - node2.first, node1.second - node2.second);
}

double dist(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2)
{
  return std::hypot(point1.x() - point2.x(), point1.y() - point2.y());
}

double dist(const geometry_msgs::PoseStamped& node1, const geometry_msgs::PoseStamped& node2)
{
  return std::hypot(node1.pose.position.x - node2.pose.position.x, node1.pose.position.y - node2.pose.position.y);
}

/**
 * @brief Calculate the angle of x-axis between the 2 nodes.
 * @param n1 Node 1
 * @param n2 Node 2
 * @return the angle of x-axis between the 2 node
 */
double angle(const Node& node1, const Node& node2)
{
  return atan2(node2.y_ - node1.y_, node2.x_ - node1.x_);
}

double angle(const std::pair<double, double>& node1, const std::pair<double, double>& node2)
{
  return atan2(node2.second - node1.second, node2.first - node1.first);
}

double angle(const geometry_msgs::PoseStamped& node1, const geometry_msgs::PoseStamped& node2)
{
  return atan2(node2.pose.position.y - node1.pose.position.y, node2.pose.position.x - node1.pose.position.x);
}
/**
 * @brief Perform modulus operation on 2π.
 * @param theta    the angle to modulu
 * @return theta_m the angle after modulus operator
 */
double mod2pi(double theta)
{
  return theta - 2.0 * M_PI * floor(theta / M_PI / 2.0);
}

/**
 * @brief Truncate the angle to the interval of -π to π.
 * @param theta    the angle to truncate
 * @return theta_t the truncated angle
 */
double pi2pi(double theta)
{
  while (theta > M_PI)
    theta -= 2.0 * M_PI;
  while (theta < -M_PI)
    theta += 2.0 * M_PI;
  return theta;
}

/**
 * @brief Formula for intersection of a line with a circle centered at the origin
 * @note  https://mathworld.wolfram.com/Circle-LineIntersection.html
 * @param p1/p2     the two point in the segment
 * @param r         the radius of circle centered at the origin
 * @return points   the intersection points of a line and the circle
 */
std::vector<std::pair<double, double>> circleSegmentIntersection(const std::pair<double, double>& p1,
                                                                 const std::pair<double, double>& p2, double r)
{
  std::vector<std::pair<double, double>> i_points;

  double x1 = p1.first;
  double x2 = p2.first;
  double y1 = p1.second;
  double y2 = p2.second;

  double dx = x2 - x1;
  double dy = y2 - y1;
  double dr2 = dx * dx + dy * dy;
  double D = x1 * y2 - x2 * y1;

  // the first element is the point within segment
  double d1 = x1 * x1 + y1 * y1;
  double d2 = x2 * x2 + y2 * y2;
  double dd = d2 - d1;

  double delta = std::sqrt(r * r * dr2 - D * D);

  if (delta >= 0)
  {
    if (delta == 0)
      i_points.emplace_back(D * dy / dr2, -D * dx / dr2);
    else
    {
      i_points.emplace_back((D * dy + std::copysign(1.0, dd) * dx * delta) / dr2,
                            (-D * dx + std::copysign(1.0, dd) * dy * delta) / dr2);
      i_points.emplace_back((D * dy - std::copysign(1.0, dd) * dx * delta) / dr2,
                            (-D * dx - std::copysign(1.0, dd) * dy * delta) / dr2);
    }
  }

  return i_points;
}
}  // namespace helper