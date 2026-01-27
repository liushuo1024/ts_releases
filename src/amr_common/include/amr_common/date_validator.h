#ifndef DATE_VALIDATOR_H
#define DATE_VALIDATOR_H

#include <chrono>
#include <ctime>
#include <iostream>

// ========== 截止日期配置 ==========
// 修改这里的日期来设置程序的有效期限
const int DEADLINE_YEAR = 2026;
const int DEADLINE_MONTH = 1;   // 1-12
const int DEADLINE_DAY = 30;
// =================================

namespace DateValidator {

/**
 * @brief 检查当前日期是否超过截止日期
 * @return true: 已过期, false: 未过期
 */
inline bool isExpired()
{
    // 获取当前系统时间
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time);

    // 转换为可比较的数字格式：YYYYMMDD
    int current_date = (local_time->tm_year + 1900) * 10000 + 
                       (local_time->tm_mon + 1) * 100 + 
                       local_time->tm_mday;
    int deadline_date = DEADLINE_YEAR * 10000 + DEADLINE_MONTH * 100 + DEADLINE_DAY;

    return current_date > deadline_date;
}

/**
 * @brief 获取当前日期字符串
 * @return 格式为 "YYYY-MM-DD" 的字符串
 */
inline std::string getCurrentDate()
{
    auto now = std::chrono::system_clock::now();
    std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm* local_time = std::localtime(&now_time);

    char buffer[11];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d", local_time);
    return std::string(buffer);
}

/**
 * @brief 检查程序是否过期，如果过期则打印错误信息并退出
 * @note 此函数会直接调用 exit()，不再返回
 */
inline void checkAndExitIfExpired()
{
    if (isExpired()) {
        // std::cerr << "Error: Program has expired!" << std::endl;
        // std::cerr << "Deadline: " << DEADLINE_YEAR << "-" << DEADLINE_MONTH << "-" << DEADLINE_DAY << std::endl;
        // std::cerr << "Current date: " << getCurrentDate() << std::endl;
        std::exit(-1);
    }
}

} // namespace DateValidator

#endif // DATE_VALIDATOR_H
