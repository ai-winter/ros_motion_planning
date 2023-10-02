/***********************************************************
 *
 * @file: local_planner.h
 * @breif: Contains the abstract local planner class
 * @author: Yang Haodong
 * @update: 2023-10-2
 * @version: 1.0
 *
 * Copyright (c) 2023， Yang Haodong
 * All rights reserved.
 * --------------------------------------------------------
 *
 **********************************************************/
#ifndef LOCAL_PLANNER_H
#define LOCAL_PLANNER_H

#define INFINITE_COST 1e10   // infinite cost
#define LETHAL_COST 253      // lethal cost
#define NEUTRAL_COST 50      // neutral cost
#define OBSTACLE_FACTOR 0.5  // obstacle factor

#include <geometry_msgs/PoseStamped.h>

#include <Eigen/Dense>

namespace local_planner
{
class LocalPlanner
{
public:
  /**
   * @brief Construct a new Local Planner object
   */
  LocalPlanner();

  /**
   * @brief Destroy the Local Planner object
   */
  virtual ~LocalPlanner() = default;

  /**
   * @brief Set or reset costmap size
   * @param nx  pixel number in costmap x direction
   * @param ny  pixel number in costmap y direction
   */
  void setSize(int nx, int ny);

  /**
   * @brief Set or reset costmap resolution
   * @param resolution  costmap resolution
   */
  void setResolution(double resolution);

  /**
   * @brief Set or reset costmap origin
   * @param origin_x  origin in costmap x direction
   * @param origin_y  origin in costmap y direction
   */
  void setOrigin(double origin_x, double origin_y);

  /**
   * @brief Set or reset lethal cost
   * @param lethal_cost lethal cost
   */
  void setLethalCost(unsigned char lethal_cost);

  /**
   * @brief Set or reset neutral cost
   * @param neutral_cost  neutral cost
   */
  void setNeutralCost(unsigned char neutral_cost);

  /**
   * @brief Set or reset obstacle factor
   * @param factor  obstacle factor
   */
  void setFactor(double factor);

  /**
   * @brief Set or reset frame name
   * @param frame_name
   */
  void setBaseFrame(std::string base_frame);
  void setMapFrame(std::string map_frame);

  /**
   * @brief Calculate distance between the 2 points.
   * @param n1        point 1
   * @param n2        point 2
   * @return distance between points
   */
  double dist(const Eigen::Vector2d& point1, const Eigen::Vector2d& point2);

  /**
   * @brief Regularize angle to [-pi, pi]
   * @param angle the angle (rad) to regularize
   */
  void regularizeAngle(double& angle);

  /**
   * @brief Get the Euler Angles from PoseStamped
   * @param ps  PoseStamped to calculate
   * @return  roll, pitch and yaw in XYZ order
   */
  Eigen::Vector3d getEulerAngles(geometry_msgs::PoseStamped& ps);

  /**
   * @brief Tranform from world map(x, y) to costmap(x, y)
   * @param mx  costmap x
   * @param my  costmap y
   * @param wx  world map x
   * @param wy  world map y
   * @return true if successfull, else false
   */
  bool worldToMap(double wx, double wy, int& mx, int& my);

protected:
  // lethal cost and neutral cost
  unsigned char lethal_cost_, neutral_cost_;

  int nx_, ny_, ns_;            // pixel number in local costmap x, y and total
  double origin_x_, origin_y_;  // local costmap origin
  double resolution_;           // local ostmap resolution
  double convert_offset_;       // offset of transform from world(x,y) to grid map(x,y)

  // obstacle factor(greater means obstacles)
  double factor_;

  // frame name of base link and map
  std::string base_frame_, map_frame_;
};
}  // namespace local_planner

#endif