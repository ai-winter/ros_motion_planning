/**
 * *********************************************************
 *
 * @file: singleton.h
 * @brief: Contains sigleton instance
 * @author: Yang Haodong
 * @date: 2024-09-22
 * @version: 3.0
 *
 * Copyright (c) 2024, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#ifndef RMP_COMMON_STRUCTURE_SINGLETON_H_
#define RMP_COMMON_STRUCTURE_SINGLETON_H_

namespace rmp
{
namespace common
{
namespace structure
{
template <typename TSingleton>
class Singleton
{
public:
  using TSingletonPtr = std::unique_ptr<TSingleton>;

private:
  Singleton() = default;
  virtual ~Singleton() = default;
  Singleton(const Singleton&) = delete;
  Singleton(Singleton&&) = delete;
  Singleton& operator=(const Singleton&) = delete;

public:
  static TSingletonPtr& Instance()
  {
    static TSingletonPtr instance = std::make_unique<TSingleton>();
    return instance;
  }
};
}  // namespace structure
}  // namespace common
}  // namespace rmp
#endif