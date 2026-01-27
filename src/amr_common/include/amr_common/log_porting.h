/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_LOG_PORTING_H_
#define amr_COMMON_INCLUDE_amr_COMMON_LOG_PORTING_H_
#include <ros/ros.h>

#include <ctime>
#include <string>

#include "amr_common/amr_enum_type.h"

#define TIME_STRING (std::string("[") + TimeInfo() + std::string("]"))

#define FILE_LINE_STRING                                                \
  (std::string(" [") +                                                  \
   std::string(__FILE__).substr(std::string(__FILE__).rfind("/") + 1) + \
   std::string(":") + std::to_string(__LINE__) + std::string("]"))

// 选择带现实时间的日志 1:true  0:false
#define LOG_WITH_TIME 1

#if LOG_WITH_TIME

#define LOG_DEBUG(format, ...)                                           \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_DEBUG(s.c_str(), " ", ##__VA_ARGS__);                            \
  } while (0)

#define LOG_DEBUG_ONCE(format, ...)                                      \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_DEBUG_ONCE(s.c_str(), " ", ##__VA_ARGS__);                       \
  } while (0)

#define LOG_DEBUG_COND(cond, format, ...)                                \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_DEBUG_COND(cond, s.c_str(), " ", ##__VA_ARGS__);                 \
  } while (0)

#define LOG_DEBUG_STREAM(args)                                           \
  do {                                                                   \
    ROS_DEBUG_STREAM(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                 << args);                               \
  } while (0)

#define LOG_DEBUG_THROTTLE(rate, format, ...)                            \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_DEBUG_THROTTLE(rate, s.c_str(), " ", ##__VA_ARGS__);             \
  } while (0)

#define LOG_DEBUG_STREAM_THROTTLE(rate, args)                               \
  do {                                                                      \
    ROS_DEBUG_STREAM_THROTTLE(                                              \
        rate, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_DEBUG_STREAM_ONCE(args)                                           \
  do {                                                                        \
    ROS_DEBUG_STREAM_ONCE(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                      << args);                               \
  } while (0)

#define LOG_DEBUG_STREAM_COND(cond, args)                                   \
  do {                                                                      \
    ROS_DEBUG_STREAM_COND(                                                  \
        cond, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_INFO(format, ...)                                            \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_INFO(s.c_str(), " ", ##__VA_ARGS__);                             \
  } while (0)

#define LOG_INFO_ONCE(format, ...)                                       \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_INFO_ONCE(s.c_str(), " ", ##__VA_ARGS__);                        \
  } while (0)

#define LOG_INFO_COND(cond, format, ...)                                 \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_INFO_COND(cond, s.c_str(), " ", ##__VA_ARGS__);                  \
  } while (0)

#define LOG_INFO_STREAM(args)                                           \
  do {                                                                  \
    ROS_INFO_STREAM(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                << args);                               \
  } while (0)

#define LOG_INFO_THROTTLE(rate, format, ...)                             \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_INFO_THROTTLE(rate, s.c_str(), " ", ##__VA_ARGS__);              \
  } while (0)

#define LOG_INFO_STREAM_THROTTLE(rate, args)                                \
  do {                                                                      \
    ROS_INFO_STREAM_THROTTLE(                                               \
        rate, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_INFO_STREAM_ONCE(args)                                           \
  do {                                                                       \
    ROS_INFO_STREAM_ONCE(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                     << args);                               \
  } while (0)

#define LOG_INFO_STREAM_COND(cond, args)                                    \
  do {                                                                      \
    ROS_INFO_STREAM_COND(                                                   \
        cond, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_WARN(format, ...)                                            \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_WARN(s.c_str(), " ", ##__VA_ARGS__);                             \
  } while (0)

#define LOG_WARN_ONCE(format, ...)                                       \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_WARN_ONCE(s.c_str(), " ", ##__VA_ARGS__);                        \
  } while (0)

#define LOG_WARN_COND(cond, format, ...)                                 \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_WARN_COND(cond, s.c_str(), " ", ##__VA_ARGS__);                  \
  } while (0)

#define LOG_WARN_STREAM(args)                                           \
  do {                                                                  \
    ROS_WARN_STREAM(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                << args);                               \
  } while (0)

#define LOG_WARN_STREAM_ONCE(args)                                           \
  do {                                                                       \
    ROS_WARN_STREAM_ONCE(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                     << args);                               \
  } while (0)

#define LOG_WARN_STREAM_COND(cond, args)                                    \
  do {                                                                      \
    ROS_WARN_STREAM_COND(                                                   \
        cond, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_WARN_THROTTLE(rate, format, ...)                             \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_WARN_THROTTLE(rate, s.c_str(), " ", ##__VA_ARGS__);              \
  } while (0)

#define LOG_WARN_STREAM_THROTTLE(rate, args)                                \
  do {                                                                      \
    ROS_WARN_STREAM_THROTTLE(                                               \
        rate, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_ERROR(format, ...)                                           \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_ERROR(s.c_str(), " ", ##__VA_ARGS__);                            \
  } while (0)

#define LOG_ERROR_ONCE(format, ...)                                      \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_ERROR_ONCE(s.c_str(), " ", ##__VA_ARGS__);                       \
  } while (0)

#define LOG_ERROR_COND(cond, format, ...)                                \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_ERROR_COND(cond, s.c_str(), " ", ##__VA_ARGS__);                 \
  } while (0)

#define LOG_ERROR_STREAM(args)                                           \
  do {                                                                   \
    ROS_ERROR_STREAM(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                 << args);                               \
  } while (0)

#define LOG_ERROR_STREAM_ONCE(args)                                           \
  do {                                                                        \
    ROS_ERROR_STREAM_ONCE(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                      << args);                               \
  } while (0)

#define LOG_ERROR_STREAM_COND(cond, args)                                   \
  do {                                                                      \
    ROS_ERROR_STREAM_COND(                                                  \
        cond, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_ERROR_THROTTLE(rate, format, ...)                            \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_ERROR_THROTTLE(rate, s.c_str(), " ", ##__VA_ARGS__);             \
  } while (0)

#define LOG_ERROR_STREAM_THROTTLE(rate, args)                               \
  do {                                                                      \
    ROS_ERROR_STREAM_THROTTLE(                                              \
        rate, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_FATAL(format, ...)                                           \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_FATAL(s.c_str(), " ", ##__VA_ARGS__);                            \
  } while (0)

#define LOG_FATAL_ONCE(format, ...)                                      \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_FATAL_ONCE(s.c_str(), " ", ##__VA_ARGS__);                       \
  } while (0)

#define LOG_FATAL_COND(cond, format, ...)                                \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_FATAL_COND(cond, s.c_str(), " ", ##__VA_ARGS__);                 \
  } while (0)

#define LOG_FATAL_STREAM(args)                                           \
  do {                                                                   \
    ROS_FATAL_STREAM(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                 << args);                               \
  } while (0)

#define LOG_FATAL_STREAM_ONCE(args)                                           \
  do {                                                                        \
    ROS_FATAL_STREAM_ONCE(TIME_STRING << FILE_LINE_STRING << std::string(" ") \
                                      << args);                               \
  } while (0)

#define LOG_FATAL_STREAM_COND(cond, args)                                   \
  do {                                                                      \
    ROS_FATAL_STREAM_COND(                                                  \
        cond, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_FATAL_THROTTLE(rate, format, ...)                            \
  do {                                                                   \
    std::string s = TIME_STRING + FILE_LINE_STRING + std::string("%s") + \
                    static_cast<std::string>(format);                    \
    ROS_FATAL_THROTTLE(rate, s.c_str(), " ", ##__VA_ARGS__);             \
  } while (0)

#define LOG_FATAL_STREAM_THROTTLE(rate, args)                               \
  do {                                                                      \
    ROS_FATAL_STREAM_THROTTLE(                                              \
        rate, TIME_STRING << FILE_LINE_STRING << std::string(" ") << args); \
  } while (0)

#define LOG_ASSERT ROS_ASSERT

#else
#define LOG_DEBUG ROS_DEBUG
#define LOG_DEBUG_ONCE ROS_DEBUG_ONCE
#define LOG_DEBUG_COND ROS_DEBUG_COND
#define LOG_DEBUG_STREAM ROS_DEBUG_STREAM
#define LOG_DEBUG_STREAM_ONCE ROS_DEBUG_STREAM_ONCE
#define LOG_DEBUG_STREAM_COND ROS_DEBUG_STREAM_COND
#define LOG_INFO ROS_INFO
#define LOG_INFO_ONCE ROS_INFO_ONCE
#define LOG_INFO_COND ROS_INFO_COND
#define LOG_INFO_STREAM ROS_INFO_STREAM
#define LOG_INFO_STREAM_ONCE ROS_INFO_STREAM_ONCE
#define LOG_INFO_STREAM_COND ROS_INFO_STREAM_COND
#define LOG_WARN ROS_WARN
#define LOG_WARN_ONCE ROS_WARN_ONCE
#define LOG_WARN_COND ROS_WARN_COND
#define LOG_WARN_STREAM ROS_WARN_STREAM
#define LOG_WARN_STREAM_ONCE ROS_WARN_STREAM_ONCE
#define LOG_WARN_STREAM_COND ROS_WARN_STREAM_COND
#define LOG_ERROR ROS_ERROR
#define LOG_ERROR_ONCE ROS_ERROR_ONCE
#define LOG_ERROR_COND ROS_ERROR_COND
#define LOG_ERROR_STREAM ROS_ERROR_STREAM
#define LOG_ERROR_STREAM_ONCE ROS_ERROR_STREAM_ONCE
#define LOG_ERROR_STREAM_COND ROS_ERROR_STREAM_COND
#define LOG_FATAL ROS_FATAL
#define LOG_FATAL_COND ROS_FATAL_COND
#define LOG_FATAL_STREAM ROS_FATAL_STREAM
#define LOG_FATAL_STREAM_ONCE ROS_FATAL_STREAM_ONCE
#define LOG_FATAL_STREAM_COND ROS_FATAL_STREAM_COND
#define LOG_ASSERT ROS_ASSERT

#endif

static std::string TimeInfo() {
  time_t timep;
  struct timeval tv;
  time(&timep);
  gettimeofday(&tv, NULL);
  char tmp[64];
  strftime(tmp, sizeof(tmp), "%Y%m%d %H:%M:%S", localtime(&timep));
  char tmp_ns[20];
  uint64_t ns = tv.tv_usec;
  int i = snprintf(tmp_ns, sizeof(ns), "%.6ld", ns);
  std::string s1(tmp);
  std::string s(s1 + ":" + tmp_ns);
  return s;
}

#endif  // amr_COMMON_INCLUDE_amr_COMMON_LOG_PORTING_H_
