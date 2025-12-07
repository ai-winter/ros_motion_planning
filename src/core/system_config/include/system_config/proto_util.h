/**
 * *********************************************************
 *
 * @file: proto_util.h
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
#include <string>

#include <google/protobuf/text_format.h>

namespace rmp {
namespace system_config {
/**
 * @brief Read a text-format protobuf message from a file.
 * @param filename Path to the text proto file.
 * @param message  Pointer to the protobuf Message to populate.
 * @return true if the file was successfully read and parsed into message, false
 * otherwise.
 */
bool readTextProtoFile(const std::string& filename, google::protobuf::Message* message);

/**
 * @brief Write a protobuf message to a file in text format.
 * @param filename Path to the output text proto file.
 * @param message  Protobuf Message to serialize.
 * @return true if the message was successfully written to the file, false otherwise.
 */
bool writeTextProtoFile(const std::string& filename,
                        const google::protobuf::Message& message);
}  // namespace system_config
}  // namespace rmp