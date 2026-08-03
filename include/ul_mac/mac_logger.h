// =============================================================================
// mac_logger.h - MAC层日志记录器
//
// 提供结构化的日志输出功能, 支持多级别日志和性能监控事件记录
// 参考 srsRAN 中 srslog 日志框架的设计理念
// =============================================================================

#pragma once

#include "ul_mac/common_types.h"
#include <mutex>
#include <fstream>
#include <chrono>
#include <ctime>

namespace ul_mac {

/// 日志级别
enum class log_level {
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    //
    TRACE
    //
};

/// MAC日志记录器
/// 线程安全, 支持控制台输出和文件记录
class mac_logger {
public:
    /// 获取单例实例
    static mac_logger& instance() {
        static mac_logger inst;
        return inst;
    }

    /// 设置日志级别
    void set_level(log_level lvl) { current_level_ = lvl; }

    /// 启用文件日志
    void enable_file_log(const std::string& filename) {
        file_stream_.open(filename, std::ios::out | std::ios::trunc);
        file_log_enabled_ = file_stream_.is_open();
    }

    /// 记录日志
    /// @param level 日志级别
    /// @param component 组件名称 (如 "SR", "BSR", "HARQ", "SCHED")
    /// @param rnti UE的RNTI (0表示系统级日志)
    /// @param tti 当前TTI
    /// @param message 日志消息
    void log(log_level level, const std::string& component,
             uint16_t rnti, uint32_t tti, const std::string& message) {
        if (level < current_level_) return;

        std::lock_guard<std::mutex> lock(mutex_);

        std::string timestamp = get_timestamp();
        std::string level_str = level_to_string(level);
        std::string rnti_str = (rnti > 0)
            ? ("UE[0x" + to_hex(rnti) + "] ")
            : "";

        std::ostringstream oss;
        oss << "[" << timestamp << "] "
            << "[" << level_str << "] "
            << "[" << component << "] "
            << rnti_str
            << format_tti(tti) << " "
            << message;

        std::string line = oss.str();

        // 控制台输出
        std::cout << line << std::endl;

        // 文件输出
        if (file_log_enabled_) {
            file_stream_ << line << std::endl;
        }
    }

    /// 便捷方法: DEBUG级别日志
    void debug(const std::string& comp, uint16_t rnti, uint32_t tti, const std::string& msg) {
        log(log_level::DEBUG, comp, rnti, tti, msg);
    }

    /// 便捷方法: INFO级别日志
    void info(const std::string& comp, uint16_t rnti, uint32_t tti, const std::string& msg) {
        log(log_level::INFO, comp, rnti, tti, msg);
    }

    /// 便捷方法: WARNING级别日志
    void warning(const std::string& comp, uint16_t rnti, uint32_t tti, const std::string& msg) {
        log(log_level::WARNING, comp, rnti, tti, msg);
    }

    /// 便捷方法: ERROR级别日志
    void error(const std::string& comp, uint16_t rnti, uint32_t tti, const std::string& msg) {
        log(log_level::ERROR, comp, rnti, tti, msg);
    }

    //
    // 便捷方法
    void trace(const std::string& comp, uint16_t rnti, uint32_t tti, const std::string& msg) {
        log(log_level::TRACE, comp, rnti, tti, msg);
    }
    //

private:
    mac_logger()
        : current_level_(log_level::INFO)
        , file_log_enabled_(false)
    {}

    std::string get_timestamp() const {
        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
        auto timer = std::chrono::system_clock::to_time_t(now);
        std::tm bt = *std::localtime(&timer);
        std::ostringstream oss;
        oss << std::put_time(&bt, "%H:%M:%S") << "."
            << std::setfill('0') << std::setw(3) << ms.count();
        return oss.str();
    }

    std::string level_to_string(log_level level) const {
        switch (level) {
            case log_level::DEBUG:   return "DEBUG";
            case log_level::INFO:    return "INFO ";
            case log_level::WARNING: return "WARN ";
            case log_level::ERROR:   return "ERROR";
            //
            case log_level::TRACE: return "TRACE";
            //
            default: return "?????";
        }
    }

    std::string to_hex(uint16_t val) const {
        std::ostringstream oss;
        oss << std::hex << std::setw(4) << std::setfill('0') << val;
        return oss.str();
    }

    log_level current_level_;
    std::mutex mutex_;
    std::ofstream file_stream_;
    bool file_log_enabled_;
};

/// 日志宏 - 简化日志调用
#define LOG_DEBUG(comp, rnti, tti, msg)  \
    ul_mac::mac_logger::instance().debug(comp, rnti, tti, msg)

#define LOG_INFO(comp, rnti, tti, msg)   \
    ul_mac::mac_logger::instance().info(comp, rnti, tti, msg)

#define LOG_WARN(comp, rnti, tti, msg)   \
    ul_mac::mac_logger::instance().warning(comp, rnti, tti, msg)

#define LOG_ERROR(comp, rnti, tti, msg)  \
    ul_mac::mac_logger::instance().error(comp, rnti, tti, msg)

#define LOG_TRACE(comp, rnti, tti, mse) \
    ul_mac::mac_logger::instance().trace(comp, rnti, tti, mes)

} // namespace ul_mac
