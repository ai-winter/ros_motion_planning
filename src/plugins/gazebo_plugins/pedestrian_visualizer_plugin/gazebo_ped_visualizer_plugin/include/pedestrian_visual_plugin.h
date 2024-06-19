/**
 * *********************************************************
 *
 * @file: pedestrian_visual_plugin.h
 * @brief: Gazebo plugin for pedestrians using social force model
 * @author: Yang Haodong
 * @date: 2023-03-15
 * @version: 1.1
 *
 * Copyright (c) 2024, Yang Haodong. 
 * All rights reserved.
 * 
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef PEDESTRIANVISUAL_GAZEBO_PLUGIN_H
#define PEDESTRIANVISUAL_GAZEBO_PLUGIN_H

// C++
#include <string>
#include <vector>
#include <algorithm>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2/utils.h>

// Gazebo
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/util/system.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo_msgs/GetModelState.h>

// message
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pedsim_msgs/TrackedPersons.h>
#include <pedsim_msgs/TrackedPerson.h>
#include <gazebo_sfm_plugin/ped_state.h>

namespace gazebo
{
class GZ_PLUGIN_VISIBLE PedestrianVisualPlugin : public ModelPlugin
{
public:
  /**
   * @brief Construct a gazebo plugin
   */
  PedestrianVisualPlugin();

  /**
   * @brief De-Construct a gazebo plugin
   */
  ~PedestrianVisualPlugin();

  /**
   * @brief Load the actor plugin.
   * @param _model  Pointer to the parent model.
   * @param _sdf    Pointer to the plugin's SDF elements.
   */
  virtual void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);

  /**
   * @brief Publish pedestrians visualization information
   */
  void publishPedVisuals();

private:
  /**
   * @brief Function that is called every update cycle.
   * @param _info Timing information.
   */
  void OnUpdate(const common::UpdateInfo& _info);

private:
  // Gazebo ROS node
  std::unique_ptr<ros::NodeHandle> node_;
  // topic publisher
  ros::Publisher ped_visual_pub_;
  // Pointer to the parent actor.
  physics::ActorPtr actor_;
  // Pointer to the world, for convenience.
  physics::WorldPtr world_;
  // Pointer to the sdf element.
  sdf::ElementPtr sdf_;
  // List of connections
  std::vector<event::ConnectionPtr> connections_;
  // Update interval
  size_t update_interval_;
  // Update counter
  size_t update_cnt_;
};
}  // namespace gazebo
#endif
