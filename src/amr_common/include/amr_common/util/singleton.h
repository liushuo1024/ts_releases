/**
 * Copyright (c) 2020 amr Inc. All rights reserved.
 */
#ifndef amr_COMMON_INCLUDE_amr_COMMON_UTIL_SINGLETON_H_
#define amr_COMMON_INCLUDE_amr_COMMON_UTIL_SINGLETON_H_
/*
可用于单例, 避免写重复的代码
比如写一个 Test 的单例:
class Test {
 private:
  static int foo_;
  DECLARE_SINGLETON(Test)
};
*/
namespace util {
#define UNUSED(param) (void)param

#define DISALLOW_CONSTRUCT(classname)\
private:                             \
  classname() {}

#define DISALLOW_COPY_AND_ASSIGN(classname) \
  classname(const classname &) = delete;    \
  classname &operator=(const classname &) = delete;

#define DECLARE_SINGLETON(classname) \
 public:                             \
  static classname &Instance() {     \
    static classname instance;       \
    return instance;                 \
  }                                  \
                                     \
 private:                            \
  DISALLOW_COPY_AND_ASSIGN(classname)
}  // namespace util
#endif  // amr_COMMON_INCLUDE_amr_COMMON_UTIL_SINGLETON_H_