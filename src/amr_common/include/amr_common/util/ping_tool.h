/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_UTIL_PING_TOOL_H_
#define amr_COMMON_INCLUDE_amr_COMMON_UTIL_PING_TOOL_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <iostream>
#include <string>

namespace util {

class PingTool {
 public:
  PingTool();
  ~PingTool();
  bool Ping(std::string ip);

 private:
  std::string GetResponse(const std::string &str_cmd);
};

}  // namespace util
#endif  // amr_COMMON_INCLUDE_amr_COMMON_UTIL_PING_TOOL_H_
