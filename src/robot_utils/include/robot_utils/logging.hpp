#pragma once

/**
 * @file logging.hpp
 * @brief 定义 `robot_utils` 包中的通用增强日志接口与宏。
 */

#include <cstdarg>
#include <cstdint>
#include <chrono>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_utils
{
/**
 * @brief 日志等级枚举，数值越大表示等级越高、越严重。
 */
enum class LogLevel : std::uint8_t
{
  Debug = 0,
  Info = 1,
  Warn = 2,
  Error = 3,
  Fatal = 4
};

/**
 * @brief 一次日志调用对应的源码位置信息。
 *
 * 这里把文件名、函数名和行号显式打包，
 * 便于后续由宏自动捕获并传给底层日志器。
 */
struct LogLocation
{
  const char * file{""};
  const char * function{""};
  int line{0};
};

/**
 * @brief 日志器运行配置。
 *
 * 当前先提供底层通用能力：
 * - 最低输出等级
 * - 是否使用 ANSI 颜色
 * - 是否显示时间戳 / 线程 ID / 源码位置
 *
 * 后续若要扩展到 ROS1/ROS2 适配层，可继续在此基础上外包一层。
 */
struct LogConfig
{
  LogLevel min_level{LogLevel::Info};
  bool enable_color{true};
  bool show_timestamp{true};
  bool show_thread_id{true};
  bool show_file_line{true};
  bool show_function{true};
  bool flush_every_line{true};
};

/**
 * @brief 简单线程安全的通用日志器。
 *
 * 设计目标：
 * 1. 不依赖 ROS 日志接口，便于作为底层公共组件存在；
 * 2. 通过宏自动补齐文件、行号、函数名；
 * 3. 支持 printf 风格格式化，便于和现有 C++ / ROS 工程习惯衔接；
 * 4. 先把核心功能搭起来，后续再决定是否做 ROS 兼容适配层。
 */
class Logger
{
public:
  /// @brief 获取全局单例日志器。
  static Logger & instance();

  /// @brief 用一整组配置覆盖当前日志器配置。
  void setConfig(const LogConfig & config);

  /// @brief 读取当前配置快照。
  LogConfig config() const;

  /// @brief 单独设置最低输出等级。
  void setMinLevel(LogLevel level);

  /// @brief 从简单配置文件中加载日志配置。
  ///
  /// 当前先支持 `key: value` 形式的轻量文本配置，不依赖额外 YAML 解析库。
  /// 这样既能满足“在 config 文件夹里集中放日志选项”的需求，
  /// 又能保持 `robot_utils` 作为底层公共模块的零 ROS 运行时依赖。
  bool loadConfigFromFile(const std::string & file_path);

  /// @brief 判断给定等级当前是否应输出。
  bool shouldLog(LogLevel level) const;

  /// @brief 把字符串解析为日志等级，成功返回 true。
  static bool parseLogLevel(const std::string & text, LogLevel & level);

  /// @brief 输出已经格式化好的日志正文。
  void log(
    LogLevel level,
    const char * tag,
    const LogLocation & location,
    const std::string & message);

  /// @brief 输出 printf 风格日志正文。
  void logf(
    LogLevel level,
    const char * tag,
    const LogLocation & location,
    const char * format,
    ...);

  /// @brief 输出 `va_list` 风格日志正文，便于内部复用。
  void vlogf(
    LogLevel level,
    const char * tag,
    const LogLocation & location,
    const char * format,
    std::va_list args);

  /// @brief 按固定时间窗节流输出 printf 风格日志。
  void logfThrottleMs(
    LogLevel level,
    const char * tag,
    const LogLocation & location,
    std::uint64_t throttle_period_ms,
    const char * format,
    ...);

private:
  /// @brief 私有构造，强制通过单例访问。
  Logger() = default;

  /// @brief 把日志等级映射成人类可读字符串。
  const char * levelToString(LogLevel level) const;

  /// @brief 根据日志等级返回 ANSI 颜色码。
  const char * levelToColor(LogLevel level) const;

  /// @brief 获取当前时间戳字符串，精确到毫秒。
  std::string currentTimestamp() const;

  /// @brief 提取路径中的文件名，避免日志里全是绝对路径。
  const char * shortFileName(const char * file_path) const;

  /// @brief 统一拼接完整日志文本。
  std::string buildLogLine(
    LogLevel level,
    const char * tag,
    const LogLocation & location,
    const std::string & message) const;

  /// @brief 实际把文本写到标准输出/错误输出。
  void writeLine(LogLevel level, const std::string & line);

  /// @brief 判断当前调用点是否允许通过节流检查。
  bool shouldLogThrottleLocked(
    const LogLocation & location,
    std::chrono::steady_clock::time_point now_time,
    std::uint64_t throttle_period_ms);

  /// @brief 去掉字符串首尾空白字符。
  static std::string trim(const std::string & text);

  /// @brief 把字符串转成小写，便于做大小写不敏感配置解析。
  static std::string toLower(const std::string & text);

  /// @brief 把字符串解析成布尔值。
  static bool parseBool(const std::string & text, bool & value);

  /// @brief 保护配置读写和输出过程，避免多线程日志交叉。
  mutable std::mutex mutex_;

  /// @brief 当前日志配置。
  LogConfig config_;

  /// @brief 按源码位置记录最近一次实际输出时间，用于实现节流日志。
  std::unordered_map<std::string, std::chrono::steady_clock::time_point> throttle_records_;
};
}  // namespace robot_utils

/// @brief 构造源码位置信息，供日志宏自动注入。
#define ROBOT_UTILS_LOG_LOCATION \
  ::robot_utils::LogLocation{__FILE__, __func__, __LINE__}

/// @brief 输出带模块标签的 DEBUG 日志。
#define ROBOT_UTILS_LOG_DEBUG_TAG(tag, format, ...) \
  ::robot_utils::Logger::instance().logf( \
  ::robot_utils::LogLevel::Debug, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  format, \
  ##__VA_ARGS__)

/// @brief 输出带模块标签的 INFO 日志。
#define ROBOT_UTILS_LOG_INFO_TAG(tag, format, ...) \
  ::robot_utils::Logger::instance().logf( \
  ::robot_utils::LogLevel::Info, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  format, \
  ##__VA_ARGS__)

/// @brief 输出带模块标签的 WARN 日志。
#define ROBOT_UTILS_LOG_WARN_TAG(tag, format, ...) \
  ::robot_utils::Logger::instance().logf( \
  ::robot_utils::LogLevel::Warn, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  format, \
  ##__VA_ARGS__)

/// @brief 输出带模块标签的 ERROR 日志。
#define ROBOT_UTILS_LOG_ERROR_TAG(tag, format, ...) \
  ::robot_utils::Logger::instance().logf( \
  ::robot_utils::LogLevel::Error, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  format, \
  ##__VA_ARGS__)

/// @brief 输出带模块标签的 FATAL 日志。
#define ROBOT_UTILS_LOG_FATAL_TAG(tag, format, ...) \
  ::robot_utils::Logger::instance().logf( \
  ::robot_utils::LogLevel::Fatal, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  format, \
  ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出 DEBUG 日志。
#define ROBOT_UTILS_LOG_DEBUG(format, ...) \
  ROBOT_UTILS_LOG_DEBUG_TAG("APP", format, ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出 INFO 日志。
#define ROBOT_UTILS_LOG_INFO(format, ...) \
  ROBOT_UTILS_LOG_INFO_TAG("APP", format, ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出 WARN 日志。
#define ROBOT_UTILS_LOG_WARN(format, ...) \
  ROBOT_UTILS_LOG_WARN_TAG("APP", format, ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出 ERROR 日志。
#define ROBOT_UTILS_LOG_ERROR(format, ...) \
  ROBOT_UTILS_LOG_ERROR_TAG("APP", format, ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出 FATAL 日志。
#define ROBOT_UTILS_LOG_FATAL(format, ...) \
  ROBOT_UTILS_LOG_FATAL_TAG("APP", format, ##__VA_ARGS__)

/// @brief 输出带模块标签的节流 WARN 日志，时间窗口单位为毫秒。
#define ROBOT_UTILS_LOG_WARN_THROTTLE_MS_TAG(tag, throttle_period_ms, format, ...) \
  ::robot_utils::Logger::instance().logfThrottleMs( \
  ::robot_utils::LogLevel::Warn, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  throttle_period_ms, \
  format, \
  ##__VA_ARGS__)

/// @brief 输出带模块标签的节流 INFO 日志，时间窗口单位为毫秒。
#define ROBOT_UTILS_LOG_INFO_THROTTLE_MS_TAG(tag, throttle_period_ms, format, ...) \
  ::robot_utils::Logger::instance().logfThrottleMs( \
  ::robot_utils::LogLevel::Info, \
  tag, \
  ROBOT_UTILS_LOG_LOCATION, \
  throttle_period_ms, \
  format, \
  ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出节流 WARN 日志。
#define ROBOT_UTILS_LOG_WARN_THROTTLE_MS(throttle_period_ms, format, ...) \
  ROBOT_UTILS_LOG_WARN_THROTTLE_MS_TAG("APP", throttle_period_ms, format, ##__VA_ARGS__)

/// @brief 默认使用 `APP` 作为模块标签输出节流 INFO 日志。
#define ROBOT_UTILS_LOG_INFO_THROTTLE_MS(throttle_period_ms, format, ...) \
  ROBOT_UTILS_LOG_INFO_THROTTLE_MS_TAG("APP", throttle_period_ms, format, ##__VA_ARGS__)
