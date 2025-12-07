/**
 * *********************************************************
 *
 * @file: system_config.cpp
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
#include "common/util/log.h"
#include "system_config/system_config.h"

namespace rmp {
namespace system_config {
/**
 * @brief Load and parse the system configuration from the file.
 */
void SystemConfig::initialize() {
  std::string file_path(__FILE__);
  size_t base_pos = file_path.rfind("/src/");
  std::string base_dir = file_path.substr(0, base_pos);
  std::string config_path = base_dir + "/system_config/system_config.pb.txt";
  if (readTextProtoFile(config_path, &config_)) {
    is_initialized_ = true;
  } else {
    R_WARN << "Read System configure file " << config_path << " failed.";
  }
}

/**
 * @brief Return the loaded system configuration, initializing on first access.
 * @return Reference to the in-memory SystemConfig protobuf.
 */
const pb::SystemConfig& SystemConfig::configure() {
  if (!is_initialized_) {
    initialize();
  }

  return config_;
}

}  // namespace system_config
}  // namespace rmp