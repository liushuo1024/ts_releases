/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_UTIL_REMOTE_SERVICE_H_
#define amr_COMMON_INCLUDE_amr_COMMON_UTIL_REMOTE_SERVICE_H_
#include <string>
#include <tuple>

#include "amr_common/log_porting.h"
#include "amr_common/util/httplib.h"

namespace util {
class RemoteServiceEntry final {
 public:
  RemoteServiceEntry() = delete;
  RemoteServiceEntry(const std::string &ip, int port, const std::string &api)
      : entry_(ip, port, api) {}

  inline const char *Ip() const { return std::get<0>(entry_).c_str(); }
  inline int Port() const { return std::get<1>(entry_); }
  inline const char *Api() const { return std::get<2>(entry_).c_str(); }

 private:
  std::tuple<std::string, int, std::string> entry_;
};

class RemoteService final {
 public:
  static std::string GetString(const RemoteServiceEntry &entry) {
    httplib::Client client(entry.Ip(), entry.Port());
    const auto &res = client.Get(entry.Api());
    if (res) {
      if (res->status != 200) {
        LOG_ERROR("http error, status: %d", res->status);
        // return false;
      }
      return res->body;
    } else {
      return std::string();
    }
  }

  static bool PostString(const RemoteServiceEntry &entry,
                         const std::string &json_str) {
    httplib::Client client(entry.Ip(), entry.Port());
    const auto &res = client.Post(entry.Api(), json_str, "application/json");
    if (res == nullptr) {
      LOG_ERROR("response empty!");
      return false;
    }

    LOG_INFO_STREAM("res: " << res->status << ", " << res->body);
    if (!std::strcmp((res->body).c_str(),
                     "{\"code\":0,\"message\":\"success\"}")) {
      LOG_INFO_STREAM("post succeed !!!");
      return true;
    } else {
      LOG_INFO_STREAM("post failed !!!");
      return false;
    }
  }

  static bool PostString(const RemoteServiceEntry &entry,
                         const std::string &json_str,
                         const std::string &content_type) {
    httplib::Client client(entry.Ip(), entry.Port());
    const auto &res = client.Post(entry.Api(), json_str, content_type.c_str());
    if (res == nullptr) {
      LOG_ERROR("response empty!");
      return false;
    }

    LOG_INFO_STREAM("res: " << res->status << ", " << res->body);
    if (!std::strcmp((res->body).c_str(),
                     "{\"code\":0,\"message\":\"success\"}")) {
      LOG_INFO_STREAM("post succeed !!!");
      return true;
    } else {
      LOG_INFO_STREAM("post failed !!!");
      return false;
    }
  }
};
}  // namespace util
#endif  // amr_COMMON_INCLUDE_amr_COMMON_REMOTE_SERVICE_H_
