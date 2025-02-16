/**
 * *********************************************************
 *
 * @file: jps_planner.h
 * @brief: Contains the Jump Point Search(JPS) planner class
 * @author: Yang Haodong
 * @date: 2024-09-30
 * @version: 2.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_PATH_PLANNER_GRAPH_PLANNER_JUMP_POINT_SEARCH_H_
#define RMP_PATH_PLANNER_GRAPH_PLANNER_JUMP_POINT_SEARCH_H_

#include <array>

#include "path_planner/path_planner.h"

namespace rmp
{
namespace path_planner
{
/**
 * @brief lass for objects that plan using the Jump Point Search(JPSPlanner) algorithm
 */
class JPSPathPlanner : public PathPlanner
{
private:
  using Node = rmp::common::structure::Node<int>;

protected:
  class JNode : public Node
  {
  public:
    /* @brief Construct a new JNode object
     * @param x   X value
     * @param y   Y value
     * @param g   Cost to get to this node
     * @param h   Heuritic cost of this node
     * @param id  JNode's id
     * @param pid JNode's parent's id
     * @param fid JNode's forced neighbor id
     */
    JNode(int x = 0, int y = 0, double g = 0.0, double h = 0.0, int id = 0, int pid = -1, int fid = 0)
      : Node(x, y, g, h, id, pid), fid_(fid)
    {
    }

    int fid() const
    {
      return fid_;
    }

    void set_fid(int fid)
    {
      fid_ = fid;
    }

  private:
    int fid_;  // forced neighbor's id
  };

  using OpenList = std::priority_queue<JNode, std::vector<JNode>, JNode::compare_cost>;

public:
  /**
   * @brief Constructor
   * @param costmap   the environment for path planning
   * @param obstacle_factor obstacle factor(greater means obstacles)
   */
  JPSPathPlanner(costmap_2d::Costmap2DROS* costmap_ros, double obstacle_factor = 1.0);

  /**
   * @brief Jump Point Search(JPS) implementation
   * @param start  start node
   * @param goal   goal node
   * @param expand containing the node been search during the process
   * @return tuple contatining a bool as to whether a path was found, and the path
   */
  bool plan(const Point3d& start, const Point3d& goal, Points3d& path, Points3d& expand);

protected:
  /**
   *  @brief jump point detection from current node (including slash and straight direction)
   *  @param  node        current node
   *  @param  open_list   open list
   */
  void _jump(const JNode& node, OpenList& open_list);

  /**
   *  @brief jump point detection in straight direction (only for left, right, top, bottom)
   *  @param  dir             direction index
   *  @param  node            current node
   *  @param  open_list       open list
   *  @return bool            whether there exists jump point of current node in given direction
   */
  bool _checkStraightLine(int dir, const JNode& node, OpenList& open_list);

  /**
   *  @brief jump point detection in slash direction (only for left-top, left-bottom, right-top, right-bottom)
   *  @param  dir             direction index
   *  @param  node            current node
   *  @param  open_list       open list
   *  @param  from_cur        detection from current if true, else from the next node
   *  @return bool            whether there exists jump point of current node in given direction
   */
  bool _checkSlashLine(int dir, const JNode& node, OpenList& open_list, bool from_cur = true);

  /**
   *  @brief force neighbor detection
   *  @param  dir             direction index
   *  @param  cur_id          current node index
   *  @param  fn_id           the set of force neighbors
   *  @return bool            whether there exists force neighbor of current node
   */
  bool _forceNeighborDetect(int dir, int cur_id, std::vector<int>& fn_id);

private:
  JNode start_, goal_;                          // start and goal node
  OpenList open_list_;                          // open list
  std::unordered_map<int, JNode> closed_list_;  // closed list

protected:
  /* neightbor obstacles (delta index)
   *          y
   *          ￪
   *     x <——
   */
  // [left, right, top, bottom, left-top, right-bottom, right-top, left-bottom]
  const std::array<int, 8> dirs_ = { 1, -1, nx_, -nx_, nx_ + 1, -nx_ - 1, nx_ - 1, -nx_ + 1 };
  const std::unordered_map<int, std::pair<int, int>> dir_to_obs_id_ = {
    { -nx_, { 0, 1 } },      // direction bottom, obstacle detection [left-right]
    { nx_, { 0, 1 } },       // direction top, obstacle detection [left-right]
    { -1, { 2, 3 } },        // direction right, obstacle detection [top-bottom]
    { 1, { 2, 3 } },         // direction left, obstacle detection [top-bottom]
    { -nx_ - 1, { 0, 2 } },  // direction right-bottom, obstacle detection [left-top]
    { nx_ + 1, { 1, 3 } },   // direction left-top, obstacle detection [right-bottom]
    { -nx_ + 1, { 1, 2 } },  // direction left-bottom, obstacle detection [right-top]
    { nx_ - 1, { 0, 3 } },   // direction right-top, obstacle detection [left-bottom]
  };
};
}  // namespace path_planner
}  // namespace rmp
#endif  // JUMP_POINT_SEARCH_H
