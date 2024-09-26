/*
 * KdTree.h
 * RVO2 Library
 *
 * SPDX-FileCopyrightText: 2008 University of North Carolina at Chapel Hill
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <https://gamma.cs.unc.edu/RVO2/>
 */

#ifndef RVO_KD_TREE_H_
#define RVO_KD_TREE_H_

/**
 * @file  KdTree.h
 * @brief Declares the KdTree class.
 */

#include <cstddef>
#include <vector>

namespace RVO {
class Agent;
class Obstacle;
class RVOSimulator;
class Vector2;

/**
 * @brief Defines k-D trees for agents and static obstacles in the simulation.
 */
class KdTree {
 private:
  class AgentTreeNode;
  class ObstacleTreeNode;

  /**
   * @brief     Constructs a k-D tree instance.
   * @param[in] simulator The simulator instance.
   */
  explicit KdTree(RVOSimulator *simulator);

  /**
   * @brief Destroys this k-D tree instance.
   */
  ~KdTree();

  /**
   * @brief Builds an agent k-D tree.
   */
  void buildAgentTree();

  /**
   * @brief     Recursive function to build an agent k-D tree.
   * @param[in] begin The beginning agent k-D tree node.
   * @param[in] end   The ending agent k-D tree node.
   * @param[in] node  The current agent k-D tree node.
   */
  void buildAgentTreeRecursive(std::size_t begin, std::size_t end,
                               std::size_t node);

  /**
   * @brief Builds an obstacle k-D tree.
   */
  void buildObstacleTree();

  /**
   * @brief     Recursive function to build an obstacle k-D tree.
   * @param[in] obstacles List of obstacles from which to build the obstacle k-D
   *                      tree.
   */
  ObstacleTreeNode *buildObstacleTreeRecursive(
      const std::vector<Obstacle *> &obstacles);

  /**
   * @brief     Computes the agent neighbors of the specified agent.
   * @param[in] agent        A pointer to the agent for which agent neighbors
   *                         are to be computed.
   * @param[in, out] rangeSq The squared range around the agent.
   */
  void computeAgentNeighbors(
      Agent *agent, float &rangeSq) const; /* NOLINT(runtime/references) */

  /**
   * @brief     Computes the obstacle neighbors of the specified agent.
   * @param[in] agent   A pointer to the agent for which obstacle neighbors are
   *                    to be computed.
   * @param[in] rangeSq The squared range around the agent.
   */
  void computeObstacleNeighbors(Agent *agent, float rangeSq) const;

  /**
   * @brief     Deletes the specified obstacle tree node.
   * @param[in] node A pointer to the obstacle tree node to be deleted.
   */
  void deleteObstacleTree(ObstacleTreeNode *node);

  /**
   * @brief         Recursive function to compute the neighbors of the specified
   *                agent.
   * @param[in]     agent   A pointer to the agent for which neighbors are to be
   *                        computed.
   * @param[in,out] rangeSq The squared range around the agent.
   * @param[in]     node    The current agent k-D tree node.
   */
  void queryAgentTreeRecursive(Agent *agent,
                               float &rangeSq, /* NOLINT(runtime/references) */
                               std::size_t node) const;

  /**
   * @brief         Recursive function to compute the neighbors of the specified
   *                obstacle.
   * @param[in]     agent   A pointer to the agent for which neighbors are to be
   *                        computed.
   * @param[in,out] rangeSq The squared range around the agent.
   * @param[in]     node    The current obstacle k-D tree node.
   */
  void queryObstacleTreeRecursive(Agent *agent, float rangeSq,
                                  const ObstacleTreeNode *node) const;

  /**
   * @brief     Queries the visibility between two points within a specified
   *            radius.
   * @param[in] vector1 The first point between which visibility is to be
   * tested.
   * @param[in] vector2 The second point between which visibility is to be
   *                    tested.
   * @param[in] radius  The radius within which visibility is to be tested.
   * @return    True if q1 and q2 are mutually visible within the radius; false
   *            otherwise.
   */
  bool queryVisibility(const Vector2 &vector1, const Vector2 &vector2,
                       float radius) const;

  /**
   * @brief     Recursive function to query the visibility between two points
   *            within a specified radius.
   * @param[in] vector1 The first point between which visibility is to be
   *                    tested.
   * @param[in] vector2 The second point between which visibility is to be
   *                    tested.
   * @param[in] radius  The radius within which visibility is to be tested.
   * @param[in] node    The current obstacle k-D tree node.
   * @return    True if q1 and q2 are mutually visible within the radius; false
   *            otherwise.
   */
  bool queryVisibilityRecursive(const Vector2 &vector1, const Vector2 &vector2,
                                float radius,
                                const ObstacleTreeNode *node) const;

  /* Not implemented. */
  KdTree(const KdTree &other);

  /* Not implemented. */
  KdTree &operator=(const KdTree &other);

  std::vector<Agent *> agents_;
  std::vector<AgentTreeNode> agentTree_;
  ObstacleTreeNode *obstacleTree_;
  RVOSimulator *simulator_;

  friend class Agent;
  friend class RVOSimulator;
};
} /* namespace RVO */

#endif /* RVO_KD_TREE_H_ */
