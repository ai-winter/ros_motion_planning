/**
 * *********************************************************
 *
 * @file: critic_function.hpp
 * @brief: Critic function used in MPPI controller.
 * @author: Zhanyu Guo
 * @date: 2026-02-21
 * @version: 1.0
 *
 * Copyright (c) 2026, Zhanyu Guo.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */

#ifndef RMP_CONTROLLER_MPPI_CRITIC_FUNCTION_HPP_
#define RMP_CONTROLLER_MPPI_CRITIC_FUNCTION_HPP_

#include <string>
#include <memory>

#include <costmap_2d/costmap_2d_ros.h>

#include "common/util/log.h"
#include "controller/critic_data.hpp"
#include "system_config/controller_protos/mppi_controller.pb.h"

namespace rmp::controller::mppi::critics {

/**
 * @class rmp::controller::mppi::critics::CollisionCost
 * @brief Utility for storing cost information
 */
struct CollisionCost {
  float cost{ 0.0f };
  bool using_footprint{ false };
};

/**
 * @class rmp::controller::mppi::critics::CriticFunction
 * @brief Abstract critic objective function to score trajectories
 */
class CriticFunction {
public:
  /**
   * @brief Constructor for mppi::critics::CriticFunction
   */
  CriticFunction() = default;

  /**
   * @brief Destructor for mppi::critics::CriticFunction
   */
  virtual ~CriticFunction() = default;

  /**
   * @brief Configure critic on bringup
   * @param name Name of plugin
   * @param costmap_ros Costmap2DROS object of environment
   * @param mppi_config MPPI controller config
   */
  void onConfigure(const std::string& name, costmap_2d::Costmap2DROS* costmap_ros,
                   pb::controller::MPPIController* mppi_config) {
    name_ = name;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros_->getCostmap();
    mppi_config_ = mppi_config;

    initialize();
  }

  /**
   * @brief Main function to score trajectory
   * @param data Critic data to use in scoring
   */
  virtual void score(CriticData& data) = 0;

  /**
   * @brief Initialize critic
   */
  virtual void initialize() = 0;

  /**
   * @brief Get name of critic
   */
  std::string getName() {
    return name_;
  }

protected:
  bool enabled_;
  std::string name_, parent_name_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  costmap_2d::Costmap2D* costmap_{ nullptr };
  pb::controller::MPPIController* mppi_config_{ nullptr };
};

}  // namespace rmp::controller::mppi::critics

#endif  // RMP_CONTROLLER_MPPI_CRITIC_FUNCTION_HPP_