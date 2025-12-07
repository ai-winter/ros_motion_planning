/**
 * *********************************************************
 *
 * @file: system_config.h
 * @brief: Contains the system configure module
 * @author: Yang Haodong
 * @date: 2025-12-5
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "system_config/proto_util.h"
#include "system_config/system_config.pb.h"
#include "common/structure/singleton.h"

namespace rmp {
namespace system_config {
class SystemConfig {
public:
  /**
   * @brief Load and parse the system configuration from the file.
   */
  void initialize();

  /**
   * @brief Return the loaded system configuration, initializing on first access.
   * @return Reference to the in-memory SystemConfig protobuf.
   */
  const pb::SystemConfig& configure();

private:
  bool is_initialized_{ false };
  pb::SystemConfig config_;
};

using SystemConfigPtr = common::structure::Singleton<SystemConfig>;

}  // namespace system_config
}  // namespace rmp