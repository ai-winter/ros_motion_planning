/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/******************************************************************************
 * Copyright 2023 Forrest. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

/**
 * @file
 */

#ifndef COMMON_LOG_H
#define COMMON_LOG_H

#include <glog/logging.h>
#include <glog/raw_logging.h>

#define DEBUG VLOG(4) << "[DEBUG] "
#define INFO LOG(INFO)
#define WARN LOG(WARNING)
#define ERROR LOG(ERROR)
#define FATAL LOG(FATAL)

// LOG_IF
#define INFO_IF(cond) LOG_IF(INFO, cond)
#define ERROR_IF(cond) LOG_IF(ERROR, cond)
// #define // CHECK(cond) // CHECK(cond)

// LOG_EVERY_N
#define INFO_EVERY(freq) LOG_EVERY_N(INFO, freq)
#define WARN_EVERY(freq) LOG_EVERY_N(WARNING, freq)
#define ERROR_EVERY(freq) LOG_EVERY_N(ERROR, freq)

#define RETURN_IF_NULL(ptr)                                                                                            \
  if (ptr == nullptr)                                                                                                  \
  {                                                                                                                    \
    WARN << #ptr << " is nullptr.";                                                                                    \
    return;                                                                                                            \
  }

#define RETURN_VAL_IF_NULL(ptr, val)                                                                                   \
  if (ptr == nullptr)                                                                                                  \
  {                                                                                                                    \
    WARN << #ptr << " is nullptr.";                                                                                    \
    return val;                                                                                                        \
  }

#define RETURN_IF(condition)                                                                                           \
  if (condition)                                                                                                       \
  {                                                                                                                    \
    WARN << #condition << " is not met.";                                                                              \
    return;                                                                                                            \
  }

#define RETURN_VAL_IF(condition, val)                                                                                  \
  if (condition)                                                                                                       \
  {                                                                                                                    \
    WARN << #condition << " is not met.";                                                                              \
    return val;                                                                                                        \
  }

#endif  // APOLLO_COMMON_LOG_H_
