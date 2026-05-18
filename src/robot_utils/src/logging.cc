#include "robot_utils/logging.hpp"

/**
 * @file logging.cc
 * @brief 实现 `robot_utils` 包中的通用增强日志器。
 */

#include <chrono>
#include <fstream>
#include <cctype>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <algorithm>
#include <sstream>
#include <thread>
#include <vector>

namespace robot_utils
{
  namespace
  {
    /// @brief 初始缓冲区大小，先覆盖绝大多数常见日志长度。
    constexpr std::size_t kInitialBufferSize = 512U;
  } // namespace

  Logger &Logger::instance()
  {
    static Logger logger;
    return logger;
  }

  void Logger::setConfig(const LogConfig &config)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_ = config;
  }

  LogConfig Logger::config() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return config_;
  }

  void Logger::setMinLevel(LogLevel level)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    config_.min_level = level;
  }

  bool Logger::loadConfigFromFile(const std::string &file_path)
  {
    std::ifstream input(file_path);
    if (!input.is_open())
    {
      return false;
    }

    LogConfig loaded_config;
    std::string line;
    while (std::getline(input, line))
    {
      // 先去掉注释和首尾空白，允许配置文件写成更接近 YAML 的样子。
      const std::size_t comment_pos = line.find('#');
      if (comment_pos != std::string::npos)
      {
        line = line.substr(0, comment_pos);
      }

      line = trim(line);
      if (line.empty())
      {
        continue;
      }

      const std::size_t separator_pos = line.find(':');
      if (separator_pos == std::string::npos)
      {
        continue;
      }

      const std::string key = trim(line.substr(0, separator_pos));
      const std::string value_text = trim(line.substr(separator_pos + 1U));
      const std::string normalized_key = toLower(key);

      if (normalized_key == "min_level")
      {
        LogLevel parsed_level;
        if (parseLogLevel(value_text, parsed_level))
        {
          loaded_config.min_level = parsed_level;
        }
        continue;
      }

      bool parsed_bool = false;
      if (normalized_key == "enable_color" && parseBool(value_text, parsed_bool))
      {
        loaded_config.enable_color = parsed_bool;
      }
      else if (normalized_key == "show_timestamp" && parseBool(value_text, parsed_bool))
      {
        loaded_config.show_timestamp = parsed_bool;
      }
      else if (normalized_key == "show_thread_id" && parseBool(value_text, parsed_bool))
      {
        loaded_config.show_thread_id = parsed_bool;
      }
      else if (normalized_key == "show_file_line" && parseBool(value_text, parsed_bool))
      {
        loaded_config.show_file_line = parsed_bool;
      }
      else if (normalized_key == "show_function" && parseBool(value_text, parsed_bool))
      {
        loaded_config.show_function = parsed_bool;
      }
      else if (normalized_key == "flush_every_line" && parseBool(value_text, parsed_bool))
      {
        loaded_config.flush_every_line = parsed_bool;
      }
    }

    setConfig(loaded_config);
    return true;
  }

  bool Logger::shouldLog(LogLevel level) const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return static_cast<int>(level) >= static_cast<int>(config_.min_level);
  }

  bool Logger::parseLogLevel(const std::string &text, LogLevel &level)
  {
    const std::string normalized = toLower(trim(text));
    if (normalized == "debug")
    {
      level = LogLevel::Debug;
      return true;
    }
    if (normalized == "info")
    {
      level = LogLevel::Info;
      return true;
    }
    if (normalized == "warn" || normalized == "warning")
    {
      level = LogLevel::Warn;
      return true;
    }
    if (normalized == "error")
    {
      level = LogLevel::Error;
      return true;
    }
    if (normalized == "fatal")
    {
      level = LogLevel::Fatal;
      return true;
    }

    return false;
  }

  void Logger::log(
      LogLevel level,
      const char *tag,
      const LogLocation &location,
      const std::string &message)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 若当前等级低于配置阈值，则直接丢弃，避免无意义格式化输出。
    if (static_cast<int>(level) < static_cast<int>(config_.min_level))
    {
      return;
    }

    const std::string line = buildLogLine(level, tag, location, message);
    writeLine(level, line);
  }

  void Logger::logf(
      LogLevel level,
      const char *tag,
      const LogLocation &location,
      const char *format,
      ...)
  {
    std::va_list args;
    va_start(args, format);
    vlogf(level, tag, location, format, args);
    va_end(args);
  }

  void Logger::logfThrottleMs(
      LogLevel level,
      const char *tag,
      const LogLocation &location,
      const std::uint64_t throttle_period_ms,
      const char *format,
      ...)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (static_cast<int>(level) < static_cast<int>(config_.min_level))
    {
      return;
    }

    const auto now_time = std::chrono::steady_clock::now();
    if (!shouldLogThrottleLocked(location, now_time, throttle_period_ms))
    {
      return;
    }

    std::va_list args;
    va_start(args, format);

    std::va_list args_copy;
    va_copy(args_copy, args);

    std::vector<char> buffer(kInitialBufferSize, '\0');
    int written = std::vsnprintf(buffer.data(), buffer.size(), format, args_copy);
    va_end(args_copy);

    if (written < 0)
    {
      va_end(args);
      const std::string line = buildLogLine(level, tag, location, "Log formatting failed.");
      writeLine(level, line);
      return;
    }

    if (static_cast<std::size_t>(written) >= buffer.size())
    {
      buffer.resize(static_cast<std::size_t>(written) + 1U, '\0');
      std::vsnprintf(buffer.data(), buffer.size(), format, args);
    }

    va_end(args);

    const std::string message(buffer.data());
    const std::string line = buildLogLine(level, tag, location, message);
    writeLine(level, line);
  }

  void Logger::vlogf(
      LogLevel level,
      const char *tag,
      const LogLocation &location,
      const char *format,
      std::va_list args)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // 先做等级过滤，避免低等级日志仍然去做格式化字符串分配。
    if (static_cast<int>(level) < static_cast<int>(config_.min_level))
    {
      return;
    }

    // `vsnprintf` 会修改 `va_list` 状态，因此需要先复制一份。
    std::va_list args_copy;
    va_copy(args_copy, args);

    std::vector<char> buffer(kInitialBufferSize, '\0');
    int written = std::vsnprintf(buffer.data(), buffer.size(), format, args_copy);
    va_end(args_copy);

    // 若初始缓冲区不足，则按实际长度重新分配一次，保证日志完整输出。
    if (written < 0)
    {
      const std::string line = buildLogLine(level, tag, location, "Log formatting failed.");
      writeLine(level, line);
      return;
    }

    if (static_cast<std::size_t>(written) >= buffer.size())
    {
      buffer.resize(static_cast<std::size_t>(written) + 1U, '\0');
      std::vsnprintf(buffer.data(), buffer.size(), format, args);
    }

    const std::string message(buffer.data());
    const std::string line = buildLogLine(level, tag, location, message);
    writeLine(level, line);
  }

  const char *Logger::levelToString(LogLevel level) const
  {
    switch (level)
    {
    case LogLevel::Debug:
      return "DEBUG";
    case LogLevel::Info:
      return "INFO";
    case LogLevel::Warn:
      return "WARN";
    case LogLevel::Error:
      return "ERROR";
    case LogLevel::Fatal:
      return "FATAL";
    default:
      return "UNKNOWN";
    }
  }

  const char *Logger::levelToColor(LogLevel level) const
  {
    switch (level)
    {
    case LogLevel::Debug:
      return "\033[36m";
    case LogLevel::Info:
      return "\033[32m";
    case LogLevel::Warn:
      return "\033[33m";
    case LogLevel::Error:
      return "\033[31m";
    case LogLevel::Fatal:
      return "\033[35m";
    default:
      return "\033[0m";
    }
  }

  std::string Logger::currentTimestamp() const
  {
    const auto now = std::chrono::system_clock::now();
    const auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
    const auto epoch_ms = now_ms.time_since_epoch().count();
    const auto ms_part = static_cast<int>(epoch_ms % 1000);

    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};

#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif

    std::ostringstream stream;
    stream << std::put_time(&local_tm, "%Y-%m-%d %H:%M:%S")
           << '.'
           << std::setw(3)
           << std::setfill('0')
           << ms_part;
    return stream.str();
  }

  const char *Logger::shortFileName(const char *file_path) const
  {
    if (!file_path)
    {
      return "";
    }

    const char *last_slash = std::strrchr(file_path, '/');
    return last_slash ? last_slash + 1 : file_path;
  }

  std::string Logger::buildLogLine(
      LogLevel level,
      const char *tag,
      const LogLocation &location,
      const std::string &message) const
  {
    std::ostringstream stream;

    // 日志前缀先给出等级，便于终端快速扫读。
    stream << '[' << levelToString(level) << ']';

    if (config_.show_timestamp)
    {
      stream << '[' << currentTimestamp() << ']';
    }

    if (tag && std::strlen(tag) > 0U)
    {
      stream << '[' << tag << ']';
    }

    if (config_.show_thread_id)
    {
      stream << "[tid=" << std::this_thread::get_id() << ']';
    }

    if (config_.show_file_line)
    {
      stream << '[' << shortFileName(location.file) << ':' << location.line << ']';
    }

    if (config_.show_function && location.function && std::strlen(location.function) > 0U)
    {
      stream << '[' << location.function << ']';
    }

    stream << ' ' << message;
    return stream.str();
  }

  void Logger::writeLine(LogLevel level, const std::string &line)
  {
    std::ostream &stream =
        (level == LogLevel::Error || level == LogLevel::Fatal) ? std::cerr : std::cout;

    if (config_.enable_color)
    {
      stream << levelToColor(level) << line << "\033[0m" << '\n';
    }
    else
    {
      stream << line << '\n';
    }

    if (config_.flush_every_line)
    {
      stream.flush();
    }
  }

  bool Logger::shouldLogThrottleLocked(
      const LogLocation &location,
      const std::chrono::steady_clock::time_point now_time,
      const std::uint64_t throttle_period_ms)
  {
    // 用 `文件名:行号:函数名` 作为节流键，语义上等价于“同一个日志调用点”。
    std::ostringstream key_stream;
    key_stream << shortFileName(location.file) << ':' << location.line << ':' << location.function;
    const std::string key = key_stream.str();

    const auto record_it = throttle_records_.find(key);
    if (record_it == throttle_records_.end())
    {
      throttle_records_[key] = now_time;
      return true;
    }

    const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                                now_time - record_it->second)
                                .count();
    if (elapsed_ms >= static_cast<long long>(throttle_period_ms))
    {
      record_it->second = now_time;
      return true;
    }

    return false;
  }

  std::string Logger::trim(const std::string &text)
  {
    const std::size_t begin = text.find_first_not_of(" \t\r\n");
    if (begin == std::string::npos)
    {
      return "";
    }

    const std::size_t end = text.find_last_not_of(" \t\r\n");
    return text.substr(begin, end - begin + 1U);
  }

  std::string Logger::toLower(const std::string &text)
  {
    std::string lowered = text;
    std::transform(
        lowered.begin(),
        lowered.end(),
        lowered.begin(),
        [](unsigned char ch)
        { return static_cast<char>(std::tolower(ch)); });
    return lowered;
  }

  bool Logger::parseBool(const std::string &text, bool &value)
  {
    const std::string normalized = toLower(trim(text));
    if (normalized == "true" || normalized == "1" || normalized == "yes" || normalized == "on")
    {
      value = true;
      return true;
    }

    if (normalized == "false" || normalized == "0" || normalized == "no" || normalized == "off")
    {
      value = false;
      return true;
    }

    return false;
  }
} // namespace robot_utils
