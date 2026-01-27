/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */

#include "amr_common/util/ping_tool.h"

namespace util {

constexpr uint16_t kMaxDataSize = 1024;

PingTool::PingTool() {}
PingTool::~PingTool() {}

bool PingTool::Ping(std::string ip) {
  std::string str_cmd = "ping " + ip + " -w 1";
  std::string str_response = GetResponse(str_cmd);
  if (str_response.find("received") != std::string::npos &&
      str_response.find(", 0 received") == std::string::npos) {
    return true;
  } else {
    return false;
  }
}

std::string PingTool::GetResponse(const std::string &str_cmd) {
  char buf[kMaxDataSize] = {0};
  FILE *pf = NULL;
  if ((pf = popen(str_cmd.c_str(), "r")) == NULL) {
    return "";
  }
  std::string str_result;
  while (fgets(buf, sizeof(buf), pf)) {
    str_result += buf;
  }

  pclose(pf);

  uint32_t i_size = str_result.size();
  if (i_size > 0 && str_result[i_size - 1] == '\n') {
    str_result = str_result.substr(0, i_size - 1);
  }
  return str_result;
}

}  // namespace util
