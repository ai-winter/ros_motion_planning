/**
 * *********************************************************
 *
 * @file: log.h
 * @brief: Contains logger
 * @author: Yang Haodong
 * @date: 2024-09-24
 * @version: 1.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include <glog/logging.h>
#include <glog/raw_logging.h>

#ifndef RMP_COMMON_UTIL_LOG_H_
#define RMP_COMMON_UTIL_LOG_H_

#define R_DEBUG VLOG(4) << "[DEBUG] "
#define R_INFO LOG(INFO)
#define R_WARN LOG(WARNING)
#define R_ERROR LOG(ERROR)
#define R_FATAL LOG(FATAL)

// LOG_IF
#define R_INFO_IF(cond) LOG_IF(INFO, cond)
#define R_ERROR_IF(cond) LOG_IF(ERROR, cond)
#define R_CHECK(cond) CHECK(cond)

// LOG_EVERY_N
#define R_INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define R_WARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define R_ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)                                                                                            \
  if (ptr == nullptr)                                                                                                  \
  {                                                                                                                    \
    R_WARN << #ptr << " is nullptr.";                                                                                  \
    return;                                                                                                            \
  }

#define RETURN_VAL_IF_NULL(ptr, val)                                                                                   \
  if (ptr == nullptr)                                                                                                  \
  {                                                                                                                    \
    R_WARN << #ptr << " is nullptr.";                                                                                  \
    return val;                                                                                                        \
  }

#define RETURN_IF(condition)                                                                                           \
  if (condition)                                                                                                       \
  {                                                                                                                    \
    R_WARN << #condition << " is not met.";                                                                            \
    return;                                                                                                            \
  }

#define RETURN_VAL_IF(condition, val)                                                                                  \
  if (condition)                                                                                                       \
  {                                                                                                                    \
    R_WARN << #condition << " is not met.";                                                                            \
    return val;                                                                                                        \
  }

namespace rmp
{
namespace common
{
namespace util
{
class LoggerInitializer
{
public:
  LoggerInitializer()
  {
    google::InitGoogleLogging("ros motion planning library logger");
    google::SetStderrLogging(google::INFO);
    FLAGS_colorlogtostderr = true;
  }

  ~LoggerInitializer()
  {
    google::ShutdownGoogleLogging();
  }
};
}  // namespace util
}  // namespace common
}  // namespace rmp
extern rmp::common::util::LoggerInitializer logger_initializer;

#endif  // APOLLO_COMMON_LOG_H_
