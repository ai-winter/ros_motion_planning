/*
 * RVOSimulator.cc
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

/**
 * @file  RVOSimulator.cc
 * @brief Defines the RVOSimulator class.
 */

#include "RVOSimulator.h"

#include <limits>
#include <utility>

#include "Agent.h"
#include "KdTree.h"
#include "Line.h"
#include "Obstacle.h"
#include "Vector2.h"

#ifdef _OPENMP
#include <omp.h>
#endif /* _OPENMP */

namespace RVO {
const std::size_t RVO_ERROR = std::numeric_limits<std::size_t>::max();

RVOSimulator::RVOSimulator()
    : defaultAgent_(NULL),
      kdTree_(new KdTree(this)),
      globalTime_(0.0F),
      timeStep_(0.0F) {}

RVOSimulator::RVOSimulator(float timeStep, float neighborDist,
                           std::size_t maxNeighbors, float timeHorizon,
                           float timeHorizonObst, float radius, float maxSpeed)
    : defaultAgent_(new Agent()),
      kdTree_(new KdTree(this)),
      globalTime_(0.0F),
      timeStep_(timeStep) {
  defaultAgent_->maxNeighbors_ = maxNeighbors;
  defaultAgent_->maxSpeed_ = maxSpeed;
  defaultAgent_->neighborDist_ = neighborDist;
  defaultAgent_->radius_ = radius;
  defaultAgent_->timeHorizon_ = timeHorizon;
  defaultAgent_->timeHorizonObst_ = timeHorizonObst;
}

RVOSimulator::RVOSimulator(float timeStep, float neighborDist,
                           std::size_t maxNeighbors, float timeHorizon,
                           float timeHorizonObst, float radius, float maxSpeed,
                           const Vector2 &velocity)
    : defaultAgent_(new Agent()),
      kdTree_(new KdTree(this)),
      globalTime_(0.0F),
      timeStep_(timeStep) {
  defaultAgent_->velocity_ = velocity;
  defaultAgent_->maxNeighbors_ = maxNeighbors;
  defaultAgent_->maxSpeed_ = maxSpeed;
  defaultAgent_->neighborDist_ = neighborDist;
  defaultAgent_->radius_ = radius;
  defaultAgent_->timeHorizon_ = timeHorizon;
  defaultAgent_->timeHorizonObst_ = timeHorizonObst;
}

RVOSimulator::~RVOSimulator() {
  delete defaultAgent_;
  delete kdTree_;

  for (std::size_t i = 0U; i < agents_.size(); ++i) {
    delete agents_[i];
  }

  for (std::size_t i = 0U; i < obstacles_.size(); ++i) {
    delete obstacles_[i];
  }
}

std::size_t RVOSimulator::addAgent(const Vector2 &position) {
  if (defaultAgent_ != NULL) {
    Agent *const agent = new Agent();
    agent->position_ = position;
    agent->velocity_ = defaultAgent_->velocity_;
    agent->id_ = agents_.size();
    agent->maxNeighbors_ = defaultAgent_->maxNeighbors_;
    agent->maxSpeed_ = defaultAgent_->maxSpeed_;
    agent->neighborDist_ = defaultAgent_->neighborDist_;
    agent->radius_ = defaultAgent_->radius_;
    agent->timeHorizon_ = defaultAgent_->timeHorizon_;
    agent->timeHorizonObst_ = defaultAgent_->timeHorizonObst_;
    agents_.push_back(agent);

    return agents_.size() - 1U;
  }

  return RVO_ERROR;
}

std::size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist,
                                   std::size_t maxNeighbors, float timeHorizon,
                                   float timeHorizonObst, float radius,
                                   float maxSpeed) {
  return addAgent(position, neighborDist, maxNeighbors, timeHorizon,
                  timeHorizonObst, radius, maxSpeed, Vector2());
}

std::size_t RVOSimulator::addAgent(const Vector2 &position, float neighborDist,
                                   std::size_t maxNeighbors, float timeHorizon,
                                   float timeHorizonObst, float radius,
                                   float maxSpeed, const Vector2 &velocity) {
  Agent *const agent = new Agent();
  agent->position_ = position;
  agent->velocity_ = velocity;
  agent->id_ = agents_.size();
  agent->maxNeighbors_ = maxNeighbors;
  agent->maxSpeed_ = maxSpeed;
  agent->neighborDist_ = neighborDist;
  agent->radius_ = radius;
  agent->timeHorizon_ = timeHorizon;
  agent->timeHorizonObst_ = timeHorizonObst;
  agents_.push_back(agent);

  return agents_.size() - 1U;
}

std::size_t RVOSimulator::addObstacle(const std::vector<Vector2> &vertices) {
  if (vertices.size() > 1U) {
    const std::size_t obstacleNo = obstacles_.size();

    for (std::size_t i = 0U; i < vertices.size(); ++i) {
      Obstacle *const obstacle = new Obstacle();
      obstacle->point_ = vertices[i];

      if (i != 0U) {
        obstacle->previous_ = obstacles_.back();
        obstacle->previous_->next_ = obstacle;
      }

      if (i == vertices.size() - 1U) {
        obstacle->next_ = obstacles_[obstacleNo];
        obstacle->next_->previous_ = obstacle;
      }

      obstacle->direction_ = normalize(
          vertices[(i == vertices.size() - 1U ? 0U : i + 1U)] - vertices[i]);

      if (vertices.size() == 2U) {
        obstacle->isConvex_ = true;
      } else {
        obstacle->isConvex_ =
            leftOf(vertices[i == 0U ? vertices.size() - 1U : i - 1U],
                   vertices[i],
                   vertices[i == vertices.size() - 1U ? 0U : i + 1U]) >= 0.0F;
      }

      obstacle->id_ = obstacles_.size();

      obstacles_.push_back(obstacle);
    }

    return obstacleNo;
  }

  return RVO_ERROR;
}

void RVOSimulator::doStep() {
  kdTree_->buildAgentTree();

#ifdef _OPENMP
#pragma omp parallel for
#endif /* _OPENMP */
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->computeNeighbors(kdTree_);
    agents_[i]->computeNewVelocity(timeStep_);
  }

#ifdef _OPENMP
#pragma omp parallel for
#endif /* _OPENMP */
  for (int i = 0; i < static_cast<int>(agents_.size()); ++i) {
    agents_[i]->update(timeStep_);
  }

  globalTime_ += timeStep_;
}

RVO::Vector2 RVOSimulator::getAgentNewSpeed(std::size_t agentNo)
{
  kdTree_->buildAgentTree();
  agents_[agentNo]->computeNeighbors(kdTree_);
  agents_[agentNo]->computeNewVelocity(timeStep_);
  return agents_[agentNo]->getNewVelocity();
}

std::size_t RVOSimulator::getAgentAgentNeighbor(std::size_t agentNo,
                                                std::size_t neighborNo) const {
  return agents_[agentNo]->agentNeighbors_[neighborNo].second->id_;
}

std::size_t RVOSimulator::getAgentMaxNeighbors(std::size_t agentNo) const {
  return agents_[agentNo]->maxNeighbors_;
}

float RVOSimulator::getAgentMaxSpeed(std::size_t agentNo) const {
  return agents_[agentNo]->maxSpeed_;
}

float RVOSimulator::getAgentNeighborDist(std::size_t agentNo) const {
  return agents_[agentNo]->neighborDist_;
}

std::size_t RVOSimulator::getAgentNumAgentNeighbors(std::size_t agentNo) const {
  return agents_[agentNo]->agentNeighbors_.size();
}

std::size_t RVOSimulator::getAgentNumObstacleNeighbors(
    std::size_t agentNo) const {
  return agents_[agentNo]->obstacleNeighbors_.size();
}

std::size_t RVOSimulator::getAgentNumORCALines(std::size_t agentNo) const {
  return agents_[agentNo]->orcaLines_.size();
}

std::size_t RVOSimulator::getAgentObstacleNeighbor(
    std::size_t agentNo, std::size_t neighborNo) const {
  return agents_[agentNo]->obstacleNeighbors_[neighborNo].second->id_;
}

const Line &RVOSimulator::getAgentORCALine(std::size_t agentNo,
                                           std::size_t lineNo) const {
  return agents_[agentNo]->orcaLines_[lineNo];
}

const Vector2 &RVOSimulator::getAgentPosition(std::size_t agentNo) const {
  return agents_[agentNo]->position_;
}

const Vector2 &RVOSimulator::getAgentPrefVelocity(std::size_t agentNo) const {
  return agents_[agentNo]->prefVelocity_;
}

float RVOSimulator::getAgentRadius(std::size_t agentNo) const {
  return agents_[agentNo]->radius_;
}

float RVOSimulator::getAgentTimeHorizon(std::size_t agentNo) const {
  return agents_[agentNo]->timeHorizon_;
}

float RVOSimulator::getAgentTimeHorizonObst(std::size_t agentNo) const {
  return agents_[agentNo]->timeHorizonObst_;
}

const Vector2 &RVOSimulator::getAgentVelocity(std::size_t agentNo) const {
  return agents_[agentNo]->velocity_;
}

const Vector2 &RVOSimulator::getObstacleVertex(std::size_t vertexNo) const {
  return obstacles_[vertexNo]->point_;
}

std::size_t RVOSimulator::getNextObstacleVertexNo(std::size_t vertexNo) const {
  return obstacles_[vertexNo]->next_->id_;
}

std::size_t RVOSimulator::getPrevObstacleVertexNo(std::size_t vertexNo) const {
  return obstacles_[vertexNo]->previous_->id_;
}

void RVOSimulator::processObstacles() { kdTree_->buildObstacleTree(); }

bool RVOSimulator::queryVisibility(const Vector2 &point1,
                                   const Vector2 &point2) const {
  return kdTree_->queryVisibility(point1, point2, 0.0F);
}

bool RVOSimulator::queryVisibility(const Vector2 &point1, const Vector2 &point2,
                                   float radius) const {
  return kdTree_->queryVisibility(point1, point2, radius);
}

void RVOSimulator::setAgentDefaults(float neighborDist,
                                    std::size_t maxNeighbors, float timeHorizon,
                                    float timeHorizonObst, float radius,
                                    float maxSpeed) {
  setAgentDefaults(neighborDist, maxNeighbors, timeHorizon, timeHorizonObst,
                   radius, maxSpeed, Vector2());
}

void RVOSimulator::setAgentDefaults(float neighborDist,
                                    std::size_t maxNeighbors, float timeHorizon,
                                    float timeHorizonObst, float radius,
                                    float maxSpeed, const Vector2 &velocity) {
  if (defaultAgent_ == NULL) {
    defaultAgent_ = new Agent();
  }

  defaultAgent_->maxNeighbors_ = maxNeighbors;
  defaultAgent_->maxSpeed_ = maxSpeed;
  defaultAgent_->neighborDist_ = neighborDist;
  defaultAgent_->radius_ = radius;
  defaultAgent_->timeHorizon_ = timeHorizon;
  defaultAgent_->timeHorizonObst_ = timeHorizonObst;
  defaultAgent_->velocity_ = velocity;
}

void RVOSimulator::setAgentMaxNeighbors(std::size_t agentNo,
                                        std::size_t maxNeighbors) {
  agents_[agentNo]->maxNeighbors_ = maxNeighbors;
}

void RVOSimulator::setAgentMaxSpeed(std::size_t agentNo, float maxSpeed) {
  agents_[agentNo]->maxSpeed_ = maxSpeed;
}

void RVOSimulator::setAgentNeighborDist(std::size_t agentNo,
                                        float neighborDist) {
  agents_[agentNo]->neighborDist_ = neighborDist;
}

void RVOSimulator::setAgentPosition(std::size_t agentNo,
                                    const Vector2 &position) {
  agents_[agentNo]->position_ = position;
}

void RVOSimulator::setAgentPrefVelocity(std::size_t agentNo,
                                        const Vector2 &prefVelocity) {
  agents_[agentNo]->prefVelocity_ = prefVelocity;
}

void RVOSimulator::setAgentRadius(std::size_t agentNo, float radius) {
  agents_[agentNo]->radius_ = radius;
}

void RVOSimulator::setAgentTimeHorizon(std::size_t agentNo, float timeHorizon) {
  agents_[agentNo]->timeHorizon_ = timeHorizon;
}

void RVOSimulator::setAgentTimeHorizonObst(std::size_t agentNo,
                                           float timeHorizonObst) {
  agents_[agentNo]->timeHorizonObst_ = timeHorizonObst;
}

void RVOSimulator::setAgentVelocity(std::size_t agentNo,
                                    const Vector2 &velocity) {
  agents_[agentNo]->velocity_ = velocity;
}
} /* namespace RVO */
