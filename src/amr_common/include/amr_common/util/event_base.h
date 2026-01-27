/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_UTIL_EVENT_BASE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_UTIL_EVENT_BASE_H_

#include <fstream>
#include <sstream>
#include <string>
#include <map>
#include <functional>

namespace util {

template <typename KeyType, typename AbstractProduct,
          class EventCallback = AbstractProduct *(*)(),
          class EventContainer = std::map<KeyType, EventCallback>>
class EventBase {
  EventContainer event_containers_;
public:
  virtual ~EventBase() {};
  virtual bool on(const KeyType &id, EventCallback cb) = 0;
  virtual bool emit(const KeyType &id, Args &&... args) = 0:
};

}  // namespace util

#endif  // amr_COMMON_INCLUDE_amr_COMMON_UTIL_LOCAL_SERVICE_H_
