/*
 * Obstacle.h
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

#ifndef RVO_OBSTACLE_H_
#define RVO_OBSTACLE_H_

/**
 * @file  Obstacle.h
 * @brief Declares the Obstacle class.
 */

#include <cstddef>

#include "Vector2.h"

namespace RVO {
/**
 * @brief Defines static obstacles in the simulation.
 */
class Obstacle {
 private:
  /**
   * @brief Constructs a static obstacle instance.
   */
  Obstacle();

  /**
   * @brief Destroys this static obstacle instance.
   */
  ~Obstacle();

  /* Not implemented. */
  Obstacle(const Obstacle &other);

  /* Not implemented. */
  Obstacle &operator=(const Obstacle &other);

  Vector2 direction_;
  Vector2 point_;
  Obstacle *next_;
  Obstacle *previous_;
  std::size_t id_;
  bool isConvex_;

  friend class Agent;
  friend class KdTree;
  friend class RVOSimulator;
};
} /* namespace RVO */

#endif /* RVO_OBSTACLE_H_ */
