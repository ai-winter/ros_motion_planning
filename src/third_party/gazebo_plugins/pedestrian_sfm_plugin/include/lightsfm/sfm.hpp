/***********************************************************************/
/**                                                                    */
/** sfm.hpp                                                            */
/**                                                                    */
/** Copyright (c) 2016, Service Robotics Lab.                          */
/**                     http://robotics.upo.es                         */
/**                                                                    */
/** All rights reserved.                                               */
/**                                                                    */
/** Authors:                                                           */
/** Ignacio Perez-Hurtado (maintainer)                                 */
/** Jesus Capitan                                                      */
/** Fernando Caballero                                                 */
/** Luis Merino                                                        */
/**                                                                    */
/** This software may be modified and distributed under the terms      */
/** of the BSD license. See the LICENSE file for details.              */
/**                                                                    */
/** http://www.opensource.org/licenses/BSD-3-Clause                    */
/**                                                                    */
/***********************************************************************/

#ifndef _SFM_HPP_
#define _SFM_HPP_

#include "map.hpp"
#include "vector2d.hpp"
#include <cmath>
#include <unordered_map>
#include <vector>

namespace sfm {
struct Forces {
  utils::Vector2d desiredForce;
  utils::Vector2d obstacleForce;
  utils::Vector2d socialForce;
  utils::Vector2d groupGazeForce;
  utils::Vector2d groupCoherenceForce;
  utils::Vector2d groupRepulsionForce;
  utils::Vector2d groupForce;
  utils::Vector2d globalForce;
  utils::Vector2d robotSocialForce;
};

struct Parameters {
  Parameters()
      : forceFactorDesired(2.0), forceFactorObstacle(10),
        forceSigmaObstacle(0.2), forceFactorSocial(2.1),
        forceFactorGroupGaze(3.0), forceFactorGroupCoherence(2.0),
        forceFactorGroupRepulsion(1.0), lambda(2.0), gamma(0.35), n(2.0),
        nPrime(3.0), relaxationTime(0.5) {}

  double forceFactorDesired;
  double forceFactorObstacle;
  double forceSigmaObstacle;
  double forceFactorSocial;
  double forceFactorGroupGaze;
  double forceFactorGroupCoherence;
  double forceFactorGroupRepulsion;
  double lambda;
  double gamma;
  double n;
  double nPrime;
  double relaxationTime;
};

struct Goal {
  utils::Vector2d center;
  double radius;
};

struct Agent {
  Agent()
      : desiredVelocity(0.6), radius(0.35), cyclicGoals(false),
        teleoperated(false), antimove(false), linearVelocity(0),
        angularVelocity(0), groupId(-1) {}

  Agent(double linearVelocity, double angularVelocity)
      : desiredVelocity(0.6), radius(0.35), cyclicGoals(false),
        teleoperated(true), antimove(false), linearVelocity(linearVelocity),
        angularVelocity(angularVelocity), groupId(-1) {}

  Agent(const utils::Vector2d &position, const utils::Angle &yaw,
        double linearVelocity, double angularVelocity)
      : position(position), yaw(yaw), desiredVelocity(0.6), radius(0.35),
        cyclicGoals(false), teleoperated(true), antimove(false),
        linearVelocity(linearVelocity), angularVelocity(angularVelocity),
        groupId(-1) {}

  void move(double dt); // only if teleoperated

  utils::Vector2d position;
  utils::Vector2d velocity;
  utils::Angle yaw;

  utils::Vector2d movement;

  double desiredVelocity;
  double radius;

  std::list<Goal> goals;
  bool cyclicGoals;

  bool teleoperated;
  bool antimove;
  double linearVelocity;
  double angularVelocity;

  int groupId;

  int id;

  Forces forces;
  Parameters params;
  std::vector<utils::Vector2d> obstacles1;
  std::vector<utils::Vector2d> obstacles2;
};

struct Group {
  utils::Vector2d center;
  std::vector<unsigned> agents;
};

class SocialForceModel {
public:
  SocialForceModel(SocialForceModel const &) = delete;
  void operator=(SocialForceModel const &) = delete;
  ~SocialForceModel() {}

  static SocialForceModel &getInstance() {
    static SocialForceModel singleton;
    return singleton;
  }

#define SFM SocialForceModel::getInstance()

  std::vector<Agent> &computeForces(std::vector<Agent> &agents,
                                    Map *map = NULL) const;
  void computeForces(Agent &me, std::vector<Agent> &agents, Map *map = NULL);
  std::vector<Agent> &updatePosition(std::vector<Agent> &agents,
                                     double dt) const;
  void updatePosition(Agent &me, double dt) const;

private:
#define PW(x) ((x) * (x))
  SocialForceModel() {}
  utils::Vector2d computeDesiredForce(Agent &agent) const;
  void computeObstacleForce(Agent &agent, Map *map) const;
  void computeSocialForce(unsigned index, std::vector<Agent> &agents) const;
  void computeSocialForce(Agent &agent, std::vector<Agent> &agents) const;
  void computeGroupForce(unsigned index,
                         const utils::Vector2d &desiredDirection,
                         std::vector<Agent> &agents,
                         const std::unordered_map<int, Group> &groups) const;
  void computeGroupForce(Agent &me, const utils::Vector2d &desiredDirection,
                         std::vector<Agent> &agents, Group &group) const;
};

inline utils::Vector2d
SocialForceModel::computeDesiredForce(Agent &agent) const {
  utils::Vector2d desiredDirection;
  if (!agent.goals.empty() &&
      (agent.goals.front().center - agent.position).norm() >
          agent.goals.front().radius) {
    utils::Vector2d diff = agent.goals.front().center - agent.position;
    desiredDirection = diff.normalized();
    agent.forces.desiredForce =
        agent.params.forceFactorDesired *
        (desiredDirection * agent.desiredVelocity - agent.velocity) /
        agent.params.relaxationTime;
    agent.antimove = false;
  } else {
    agent.forces.desiredForce = -agent.velocity / agent.params.relaxationTime;
    agent.antimove = true;
  }
  return desiredDirection;
}

inline void SocialForceModel::computeObstacleForce(Agent &agent,
                                                   Map *map) const {

  // Initially, the obstacles were expected to be in robot local frame. We have
  // replaced it to work in the same frame of the other forces.
  //   if (agent.obstacles1.size() > 0 || agent.obstacles2.size() > 0) {
  //     agent.forces.obstacleForce.set(0, 0);
  //     for (unsigned i = 0; i < agent.obstacles1.size(); i++) {
  //       double distance = agent.obstacles1[i].norm() - agent.radius;
  //       agent.forces.obstacleForce +=
  //           agent.params.forceFactorObstacle *
  //           std::exp(-distance / agent.params.forceSigmaObstacle) *
  //           (-agent.obstacles1[i]).normalized();
  //     }
  //     for (unsigned i = 0; i < agent.obstacles2.size(); i++) {
  //       double distance = agent.obstacles2[i].norm() - agent.radius;
  //       agent.forces.obstacleForce +=
  //           agent.params.forceFactorObstacle *
  //           std::exp(-distance / agent.params.forceSigmaObstacle) *
  //           (-agent.obstacles2[i]).normalized();
  //     }
  //     agent.forces.obstacleForce /=
  //         (double)(agent.obstacles1.size() + agent.obstacles2.size());
  if (agent.obstacles1.size() > 0 || agent.obstacles2.size() > 0) {
    agent.forces.obstacleForce.set(0, 0);
    for (unsigned i = 0; i < agent.obstacles1.size(); i++) {
      utils::Vector2d minDiff = agent.position - agent.obstacles1[i];
      double distance = minDiff.norm() - agent.radius;
      agent.forces.obstacleForce +=
          agent.params.forceFactorObstacle *
          std::exp(-distance / agent.params.forceSigmaObstacle) *
          minDiff.normalized();
    }
    for (unsigned i = 0; i < agent.obstacles2.size(); i++) {
      utils::Vector2d minDiff = agent.position - agent.obstacles2[i];
      double distance = minDiff.norm() - agent.radius;
      agent.forces.obstacleForce +=
          agent.params.forceFactorObstacle *
          std::exp(-distance / agent.params.forceSigmaObstacle) *
          minDiff.normalized();
    }
    agent.forces.obstacleForce /=
        (double)(agent.obstacles1.size() + agent.obstacles2.size());
  } else if (map != NULL) {
    const Map::Obstacle &obs = map->getNearestObstacle(agent.position);
    utils::Vector2d minDiff = agent.position - obs.position;
    double distance = minDiff.norm() - agent.radius;
    agent.forces.obstacleForce =
        agent.params.forceFactorObstacle *
        std::exp(-distance / agent.params.forceSigmaObstacle) *
        minDiff.normalized();
  } else {
    agent.forces.obstacleForce.set(0, 0);
  }
}

inline void
SocialForceModel::computeSocialForce(unsigned index,
                                     std::vector<Agent> &agents) const {
  Agent &agent = agents[index];
  agent.forces.socialForce.set(0, 0);
  for (unsigned i = 0; i < agents.size(); i++) {
    if (i == index) {
      continue;
    }
    utils::Vector2d diff = agents[i].position - agent.position;
    utils::Vector2d diffDirection = diff.normalized();
    utils::Vector2d velDiff = agent.velocity - agents[i].velocity;
    utils::Vector2d interactionVector =
        agent.params.lambda * velDiff + diffDirection;
    double interactionLength = interactionVector.norm();
    utils::Vector2d interactionDirection =
        interactionVector / interactionLength;
    utils::Angle theta = interactionDirection.angleTo(diffDirection);
    double B = agent.params.gamma * interactionLength;
    double thetaRad = theta.toRadian();
    double forceVelocityAmount =
        -std::exp(-diff.norm() / B - PW(agent.params.nPrime * B * thetaRad));
    double forceAngleAmount =
        -theta.sign() *
        std::exp(-diff.norm() / B - PW(agent.params.n * B * thetaRad));
    utils::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
    utils::Vector2d forceAngle =
        forceAngleAmount * interactionDirection.leftNormalVector();
    agent.forces.socialForce +=
        agent.params.forceFactorSocial * (forceVelocity + forceAngle);
    if (i == 0) {
      agent.forces.robotSocialForce =
          agent.params.forceFactorSocial * (forceVelocity + forceAngle);
    }
  }
}

inline void
SocialForceModel::computeSocialForce(Agent &me,
                                     std::vector<Agent> &agents) const {
  // Agent& agent = agents[index];
  me.forces.socialForce.set(0, 0);
  for (unsigned i = 0; i < agents.size(); i++) {
    if (agents[i].id == me.id) {
      continue;
    }
    utils::Vector2d diff = agents[i].position - me.position;
    utils::Vector2d diffDirection = diff.normalized();
    utils::Vector2d velDiff = me.velocity - agents[i].velocity;
    utils::Vector2d interactionVector =
        me.params.lambda * velDiff + diffDirection;
    double interactionLength = interactionVector.norm();
    utils::Vector2d interactionDirection =
        interactionVector / interactionLength;
    utils::Angle theta = interactionDirection.angleTo(diffDirection);
    double B = me.params.gamma * interactionLength;
    double thetaRad = theta.toRadian();
    double forceVelocityAmount =
        -std::exp(-diff.norm() / B - PW(me.params.nPrime * B * thetaRad));
    double forceAngleAmount =
        -theta.sign() *
        std::exp(-diff.norm() / B - PW(me.params.n * B * thetaRad));
    utils::Vector2d forceVelocity = forceVelocityAmount * interactionDirection;
    utils::Vector2d forceAngle =
        forceAngleAmount * interactionDirection.leftNormalVector();
    me.forces.socialForce +=
        me.params.forceFactorSocial * (forceVelocity + forceAngle);
    // if (i == 0)
    //{
    //  agent.forces.robotSocialForce =
    //      agent.params.forceFactorSocial * (forceVelocity + forceAngle);
    //}
  }
}

inline void SocialForceModel::computeGroupForce(
    unsigned index, const utils::Vector2d &desiredDirection,
    std::vector<Agent> &agents,
    const std::unordered_map<int, Group> &groups) const {
  Agent &agent = agents[index];
  agent.forces.groupForce.set(0, 0);
  agent.forces.groupGazeForce.set(0, 0);
  agent.forces.groupCoherenceForce.set(0, 0);
  agent.forces.groupRepulsionForce.set(0, 0);
  if (groups.count(agent.groupId) == 0 ||
      groups.at(agent.groupId).agents.size() < 2) {
    return;
  }
  const Group &group = groups.at(agent.groupId);

  // Gaze force
  utils::Vector2d com = group.center;
  com = (1 / (double)(group.agents.size() - 1)) *
        (group.agents.size() * com - agent.position);

  utils::Vector2d relativeCom = com - agent.position;
  utils::Angle visionAngle = utils::Angle::fromDegree(90);
  double elementProduct = desiredDirection.dot(relativeCom);
  utils::Angle comAngle = utils::Angle::fromRadian(std::acos(
      elementProduct / (desiredDirection.norm() * relativeCom.norm())));
  if (comAngle > visionAngle) {
#ifdef _PAPER_VERSION_
    utils::Angle necessaryRotation = comAngle - visionAngle;
    agent.forces.groupGazeForce =
        -necessaryRotation.toRadian() * desiredDirection;
#else
    double desiredDirectionSquared = desiredDirection.squaredNorm();
    double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
    agent.forces.groupGazeForce = desiredDirectionDistance * desiredDirection;
#endif
    agent.forces.groupGazeForce *= agent.params.forceFactorGroupGaze;
  }

  // Coherence force
  com = group.center;
  relativeCom = com - agent.position;
  double distance = relativeCom.norm();
  double maxDistance = ((double)group.agents.size() - 1) / 2;
#ifdef _PAPER_VERSION_
  if (distance >= maxDistance) {
    agent.forces.groupCoherenceForce = relativeCom.normalized();
    agent.forces.groupCoherenceForce *= agent.params.forceFactorGroupCoherence;
  }
#else
  agent.forces.groupCoherenceForce = relativeCom;
  double softenedFactor = agent.params.forceFactorGroupCoherence *
                          (std::tanh(distance - maxDistance) + 1) / 2;
  agent.forces.groupCoherenceForce *= softenedFactor;
#endif

  // Repulsion Force
  for (unsigned i = 0; i < group.agents.size(); i++) {
    if (index == group.agents[i]) {
      continue;
    }
    utils::Vector2d diff = agent.position - agents.at(group.agents[i]).position;
    if (diff.norm() < agent.radius + agents.at(group.agents[i]).radius) {
      agent.forces.groupRepulsionForce += diff;
    }
  }
  agent.forces.groupRepulsionForce *= agent.params.forceFactorGroupRepulsion;

  // Group Force
  agent.forces.groupForce = agent.forces.groupGazeForce +
                            agent.forces.groupCoherenceForce +
                            agent.forces.groupRepulsionForce;
}

inline void SocialForceModel::computeGroupForce(
    Agent &me, const utils::Vector2d &desiredDirection,
    std::vector<Agent> &agents, Group &group) const {
  // Agent& agent = agents[index];
  me.forces.groupForce.set(0, 0);
  me.forces.groupGazeForce.set(0, 0);
  me.forces.groupCoherenceForce.set(0, 0);
  me.forces.groupRepulsionForce.set(0, 0);
  if (group.agents.size() < 2) {
    return;
  }

  // Gaze force
  utils::Vector2d com = group.center;
  com = (1 / (double)(group.agents.size() - 1)) *
        (group.agents.size() * com - me.position);

  utils::Vector2d relativeCom = com - me.position;
  utils::Angle visionAngle = utils::Angle::fromDegree(90);
  double elementProduct = desiredDirection.dot(relativeCom);
  utils::Angle comAngle = utils::Angle::fromRadian(std::acos(
      elementProduct / (desiredDirection.norm() * relativeCom.norm())));
  if (comAngle > visionAngle) {
#ifdef _PAPER_VERSION_
    utils::Angle necessaryRotation = comAngle - visionAngle;
    me.forces.groupGazeForce = -necessaryRotation.toRadian() * desiredDirection;
#else
    double desiredDirectionSquared = desiredDirection.squaredNorm();
    double desiredDirectionDistance = elementProduct / desiredDirectionSquared;
    me.forces.groupGazeForce = desiredDirectionDistance * desiredDirection;
#endif
    me.forces.groupGazeForce *= me.params.forceFactorGroupGaze;
  }

  // Coherence force
  com = group.center;
  relativeCom = com - me.position;
  double distance = relativeCom.norm();
  double maxDistance = ((double)group.agents.size() - 1) / 2;
#ifdef _PAPER_VERSION_
  if (distance >= maxDistance) {
    me.forces.groupCoherenceForce = relativeCom.normalized();
    me.forces.groupCoherenceForce *= me.params.forceFactorGroupCoherence;
  }
#else
  me.forces.groupCoherenceForce = relativeCom;
  double softenedFactor = me.params.forceFactorGroupCoherence *
                          (std::tanh(distance - maxDistance) + 1) / 2;
  me.forces.groupCoherenceForce *= softenedFactor;
#endif

  // Repulsion Force
  // Index 0 -> me
  for (unsigned i = 1; i < group.agents.size(); i++) {

    utils::Vector2d diff = me.position - agents.at(group.agents[i]).position;
    if (diff.norm() < me.radius + agents.at(group.agents[i]).radius) {
      me.forces.groupRepulsionForce += diff;
    }
  }
  me.forces.groupRepulsionForce *= me.params.forceFactorGroupRepulsion;

  // Group Force
  me.forces.groupForce = me.forces.groupGazeForce +
                         me.forces.groupCoherenceForce +
                         me.forces.groupRepulsionForce;
}

inline std::vector<Agent> &
SocialForceModel::computeForces(std::vector<Agent> &agents, Map *map) const {
  std::unordered_map<int, Group> groups;
  for (unsigned i = 0; i < agents.size(); i++) {
    if (agents[i].groupId < 0) {
      continue;
    }
    groups[agents[i].groupId].agents.push_back(i);
    groups[agents[i].groupId].center += agents[i].position;
  }
  for (auto it = groups.begin(); it != groups.end(); ++it) {
    it->second.center /= (double)(it->second.agents.size());
  }

  for (unsigned i = 0; i < agents.size(); i++) {
    utils::Vector2d desiredDirection = computeDesiredForce(agents[i]);
    computeObstacleForce(agents[i], map);
    computeSocialForce(i, agents);
    computeGroupForce(i, desiredDirection, agents, groups);
    agents[i].forces.globalForce =
        agents[i].forces.desiredForce + agents[i].forces.socialForce +
        agents[i].forces.obstacleForce + agents[i].forces.groupForce;
  }
  return agents;
}

inline void SocialForceModel::computeForces(Agent &me,
                                            std::vector<Agent> &agents,
                                            Map *map) {
  // form the group
  Group mygroup;
  if (me.groupId != -1) {
    mygroup.agents.push_back(me.id);
    mygroup.center = me.position;
    for (unsigned i = 0; i < agents.size(); i++) {
      if (agents[i].id == me.id) {
        continue;
      }
      if (agents[i].groupId == me.groupId) {
        mygroup.agents.push_back(i);
        mygroup.center += agents[i].position;
      }
    }
    mygroup.center /= (double)mygroup.agents.size();
  }

  // Compute agent's forces
  utils::Vector2d desiredDirection = computeDesiredForce(me);
  computeObstacleForce(me, map);
  computeSocialForce(me, agents);
  computeGroupForce(me, desiredDirection, agents, mygroup);
  me.forces.globalForce = me.forces.desiredForce + me.forces.socialForce +
                          me.forces.obstacleForce + me.forces.groupForce;
}

inline void Agent::move(double dt) {
  double imd = linearVelocity * dt;
  utils::Vector2d inc(
      imd * std::cos(yaw.toRadian() + angularVelocity * dt * 0.5),
      imd * std::sin(yaw.toRadian() + angularVelocity * dt * 0.5));
  yaw += utils::Angle::fromRadian(angularVelocity * dt);
  position += inc;
  velocity.set(linearVelocity * yaw.cos(), linearVelocity * yaw.sin());
}

inline std::vector<Agent> &
SocialForceModel::updatePosition(std::vector<Agent> &agents, double dt) const {
  for (unsigned i = 0; i < agents.size(); i++) {
    utils::Vector2d initPos = agents[i].position;
    if (agents[i].teleoperated) {
      double imd = agents[i].linearVelocity * dt;
      utils::Vector2d inc(imd * std::cos(agents[i].yaw.toRadian() +
                                         agents[i].angularVelocity * dt * 0.5),
                          imd * std::sin(agents[i].yaw.toRadian() +
                                         agents[i].angularVelocity * dt * 0.5));
      agents[i].yaw += utils::Angle::fromRadian(agents[i].angularVelocity * dt);
      agents[i].position += inc;
      agents[i].velocity.set(agents[i].linearVelocity * agents[i].yaw.cos(),
                             agents[i].linearVelocity * agents[i].yaw.sin());
    } else {
      agents[i].velocity += agents[i].forces.globalForce * dt;
      if (agents[i].velocity.norm() > agents[i].desiredVelocity) {
        agents[i].velocity.normalize();
        agents[i].velocity *= agents[i].desiredVelocity;
      }
      agents[i].yaw = agents[i].velocity.angle();
      agents[i].position += agents[i].velocity * dt;
    }
    agents[i].movement = agents[i].position - initPos;
    if (!agents[i].goals.empty() &&
        (agents[i].goals.front().center - agents[i].position).norm() <=
            agents[i].goals.front().radius) {
      Goal g = agents[i].goals.front();
      agents[i].goals.pop_front();
      if (agents[i].cyclicGoals) {
        agents[i].goals.push_back(g);
      }
    }
  }
  return agents;
}

inline void SocialForceModel::updatePosition(Agent &agent, double dt) const {

  utils::Vector2d initPos = agent.position;
  utils::Angle initYaw = agent.yaw;

  agent.velocity += agent.forces.globalForce * dt;
  if (agent.velocity.norm() > agent.desiredVelocity) {
    agent.velocity.normalize();
    agent.velocity *= agent.desiredVelocity;
  }
  agent.yaw = agent.velocity.angle();
  agent.position += agent.velocity * dt;

  agent.linearVelocity = agent.velocity.norm();
  agent.angularVelocity = (agent.yaw - initYaw).toRadian() / dt;

  agent.movement = agent.position - initPos;
  if (!agent.goals.empty() &&
      (agent.goals.front().center - agent.position).norm() <=
          agent.goals.front().radius) {
    Goal g = agent.goals.front();
    agent.goals.pop_front();
    if (agent.cyclicGoals) {
      agent.goals.push_back(g);
    }
  }
}

} // namespace sfm
#endif
