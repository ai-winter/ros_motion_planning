/**
 * *********************************************************
 *
 * @file: voronoi_planner.cpp
 * @brief: Contains the Voronoi-based planner class
 * @author: Yang Haodong
 * @date: 2023-07-21
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <algorithm>
#include <queue>
#include <cmath>
#include <unordered_set>

#include "path_planner/graph_planner/voronoi_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief Construct a new Voronoi-based planning object
 * @param costmap   the environment for path planning
 * @param circumscribed_radius  the circumscribed radius of robot
 * @param obstacle_factor obstacle factor(greater means obstacles)
 */
VoronoiPathPlanner::VoronoiPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double circumscribed_radius,
                                       double obstacle_factor)
  : PathPlanner(costmap_ros, obstacle_factor), circumscribed_radius_(circumscribed_radius)
{
  voronoi_diagram_ = new VoronoiData*[nx_];
  for (int i = 0; i < nx_; i++)
    voronoi_diagram_[i] = new VoronoiData[ny_];
}

VoronoiPathPlanner::~VoronoiPathPlanner()
{
  for (int i = 0; i < nx_; i++)
    delete[] voronoi_diagram_[i];
  delete[] voronoi_diagram_;
}

/**
 * @brief Voronoi-based planning implementation
 * @param start         start node
 * @param goal          goal node
 * @param path          optimal path consists of Node
 * @param expand        containing the node been search during the process
 * @return  true if path found, else false
 */
bool VoronoiPathPlanner::plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand)
{
  // update voronoi diagram
  _updateVoronoi();
  for (int j = 0; j < ny_; j++)
  {
    for (int i = 0; i < nx_; i++)
    {
      voronoi_diagram_[i][j].dist = voronoi_.getDistance(i, j) * costmap_->getResolution();
      voronoi_diagram_[i][j].is_voronoi = voronoi_.isVoronoi(i, j);
    }
  }

  double m_start_x, m_start_y, m_goal_x, m_goal_y;
  if ((!validityCheck(start.x(), start.y(), m_start_x, m_start_y)) ||
      (!validityCheck(goal.x(), goal.y(), m_goal_x, m_goal_y)))
  {
    return false;
  }
  Node start_node(m_start_x, m_start_y);
  Node goal_node(m_goal_x, m_goal_y);
  start_node.set_id(grid2Index(start_node.x(), start_node.y()));
  goal_node.set_id(grid2Index(goal_node.x(), goal_node.y()));

  // clear vector
  path.clear();
  expand.clear();

  // start/goal to Voronoi Diagram, shortest path in Voronoi Diagram
  std::vector<Node> path_s, path_g, path_v;

  // start/goal point in Voronoi Diagram
  Node v_start, v_goal;

  if (!searchPathWithVoronoi(start_node, goal_node, path_s, &v_start))
    return false;

  if (!searchPathWithVoronoi(goal_node, start_node, path_g, &v_goal))
    return false;
  std::reverse(path_g.begin(), path_g.end());

  if (!searchPathWithVoronoi(v_start, v_goal, path_v))
    return false;

  path_g.insert(path_g.end(), path_v.begin(), path_v.end());
  path_g.insert(path_g.end(), path_s.begin(), path_s.end());
  for (auto iter = path_g.rbegin(); iter != path_g.rend(); iter++)
  {
    // convert to world frame
    double wx, wy;
    costmap_->mapToWorld(iter->x(), iter->y(), wx, wy);
    path.emplace_back(wx, wy);
  }

  return true;
}

/**
 * @brief search the shortest path from start to VD, or search the shortest path in VD
 * @param start         start node
 * @param goal          goal node
 * @param v_goal        the voronoi node in VD which is closest to start node
 * @param path          shortest path from start to VD
 * @return  true if path found, else false
 */
bool VoronoiPathPlanner::searchPathWithVoronoi(const Node& start, const Node& goal, std::vector<Node>& path,
                                               Node* v_goal)
{
  path.clear();

  // open list and closed list
  std::priority_queue<Node, std::vector<Node>, Node::compare_cost> open_list;
  std::unordered_map<int, Node> closed_list;

  open_list.push(start);

  while (!open_list.empty())
  {
    // pop current node from open list
    auto current = open_list.top();
    open_list.pop();

    // current node does not exist in closed list
    if (closed_list.find(current.id()) != closed_list.end())
      continue;

    closed_list.insert(std::make_pair(current.id(), current));

    // goal found
    if ((current == goal) ||
        (v_goal == nullptr ?
             false :
             voronoi_diagram_[static_cast<unsigned int>(current.x())][static_cast<unsigned int>(current.y())]
                 .is_voronoi))
    {
      path = _convertClosedListToPath<Node>(closed_list, start, current);
      if (v_goal != nullptr)
      {
        v_goal->set_x(current.x());
        v_goal->set_y(current.y());
        v_goal->set_id(current.id());
      }
      return true;
    }

    // explore neighbor of current node
    for (const auto& m : motions)
    {
      auto node_new = current + m;

      // current node do not exist in closed list
      if (closed_list.find(node_new.id()) != closed_list.end())
        continue;

      // explore a new node
      node_new.set_g(current.g() + m.g());
      node_new.set_id(grid2Index(node_new.x(), node_new.y()));
      node_new.set_pid(current.id());

      // next node hit the boundary or obstacle
      if ((node_new.id() < 0) || (node_new.id() >= map_size_) ||
          (voronoi_diagram_[static_cast<unsigned int>(node_new.x())][static_cast<unsigned int>(node_new.y())].dist <
           circumscribed_radius_))
        continue;

      // search in VD
      if ((v_goal == nullptr) &&
          (!voronoi_diagram_[static_cast<unsigned int>(node_new.x())][static_cast<unsigned int>(node_new.y())]
                .is_voronoi))
        continue;

      node_new.set_h(std::hypot(node_new.x() - goal.x(), node_new.y() - goal.y()));

      open_list.push(node_new);
    }
  }
  return false;
}

/**
 * @brief update voronoi object using costmap ROS wrapper
 */
void VoronoiPathPlanner::_updateVoronoi()
{
  bool voronoi_layer_exist = false;
  for (auto layer = costmap_ros_->getLayeredCostmap()->getPlugins()->begin();
       layer != costmap_ros_->getLayeredCostmap()->getPlugins()->end(); ++layer)
  {
    boost::shared_ptr<costmap_2d::VoronoiLayer> voronoi_layer =
        boost::dynamic_pointer_cast<costmap_2d::VoronoiLayer>(*layer);
    if (voronoi_layer)
    {
      voronoi_layer_exist = true;
      boost::unique_lock<boost::mutex> lock(voronoi_layer->getMutex());
      voronoi_ = voronoi_layer->getVoronoi();
      break;
    }
  }
  if (!voronoi_layer_exist)
  {
    ROS_ERROR("Failed to get a Voronoi layer for potentional application.");
  }
}

}  // namespace path_planner
}  // namespace rmp