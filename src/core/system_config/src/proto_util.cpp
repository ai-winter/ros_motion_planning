/**
 * *********************************************************
 *
 * @file: proto_util.cpp
 * @brief: Contains the tool functions to operate google protobuf
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
#include <fstream>
#include <filesystem>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/util/json_util.h>

#include "common/util/log.h"
#include "system_config/proto_util.h"

namespace rmp {
namespace system_config {
/**
 * @brief Read a text-format protobuf message from a file.
 * @param filename Path to the text proto file.
 * @param message  Pointer to the protobuf Message to populate.
 * @return true if the file was successfully read and parsed into message, false
 * otherwise.
 */
bool readTextProtoFile(const std::string& filename, google::protobuf::Message* message) {
  std::error_code error;
  if (!std::filesystem::is_regular_file(filename, error)) {
    R_WARN << filename << " is not regular file, error code: " << error.value();
    return false;
  }
  std::ifstream input(filename);
  if (!input.is_open()) {
    R_WARN << filename << " is not open";
    return false;
  }
  google::protobuf::io::IstreamInputStream file_input(&input);
  if (!google::protobuf::TextFormat::Parse(&file_input, message)) {
    R_WARN << "cannot parse " << filename;
    return false;
  }
  return true;
}

/**
 * @brief Write a protobuf message to a file in text format.
 * @param filename Path to the output text proto file.
 * @param message  Protobuf Message to serialize.
 * @return true if the message was successfully written to the file, false otherwise.
 */
bool writeTextProtoFile(const std::string& filename,
                        const google::protobuf::Message& message) {
  std::ofstream output(filename);
  if (!output.is_open()) {
    return false;
  }
  google::protobuf::io::OstreamOutputStream file_output(&output);
  return google::protobuf::TextFormat::Print(message, &file_output);
}
}  // namespace system_config
}  // namespace rmp