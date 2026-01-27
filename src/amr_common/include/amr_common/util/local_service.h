/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_UTIL_LOCAL_SERVICE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_UTIL_LOCAL_SERVICE_H_
#include <fstream>
#include <sstream>
#include <string>

namespace util {
class LocalService final {
 public:
  static std::string GetStringFromFile(const std::string &file_path) {
    std::ifstream ifs(file_path);
    if (!ifs.is_open()) {
      ifs.close();
      return std::string();
    }

    std::stringstream ss;
    ss << ifs.rdbuf();
    ifs.close();
    return ss.str();
  }

  static bool SaveStringToFile(const std::string &file_path,
                               const std::string data) {
    std::ofstream ofs(file_path);
    if (!ofs.is_open()) {
      ofs.close();
      return false;
    }
    ofs << data;
    ofs.close();
    return true;
  }
};
}  // namespace util

#endif  // amr_COMMON_INCLUDE_amr_COMMON_UTIL_LOCAL_SERVICE_H_
