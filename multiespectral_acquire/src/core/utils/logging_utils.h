/**
 * Logging utils interface to be used with non-ROS components to maintain compatibility and avoid ROS dependencies.
 */
#ifndef LOGGING_UTILS_H
#define LOGGING_UTILS_H

#define ERROR_F "\x1b[31m"
#define SUCCEED_F "\x1b[32m"
#define WARN_F "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define RESET_F "\x1b[0m"


class LoggerStreamHelper {
public:
    LoggerStreamHelper(std::function<void(const std::string&)> cb) : cb_(cb) {}
    ~LoggerStreamHelper() { cb_(oss_.str()); }

    template<typename T>
    LoggerStreamHelper& operator<<(const T& val) {
        oss_ << val;
        return *this;
    }

    // Soporte para manipuladores de ostream como std::endl
    LoggerStreamHelper& operator<<(std::ostream& (*manip)(std::ostream&)) {
        manip(oss_);
        return *this;
    }

private:
    std::ostringstream oss_;
    std::function<void(const std::string&)> cb_;
};


class Logger {
public:
    virtual void debug(const std::string& msg) = 0;
    virtual void info(const std::string& msg) = 0;
    virtual void warn(const std::string& msg) = 0;
    virtual void error(const std::string& msg) = 0;
    virtual void fatal(const std::string& msg) = 0;

    virtual LoggerStreamHelper debug_stream() { return LoggerStreamHelper([this](const std::string& s){ debug(s); }); }
    virtual LoggerStreamHelper info_stream() { return LoggerStreamHelper([this](const std::string& s){ info(s); }); }
    virtual LoggerStreamHelper warn_stream() { return LoggerStreamHelper([this](const std::string& s){ warn(s); }); }
    virtual LoggerStreamHelper error_stream() { return LoggerStreamHelper([this](const std::string& s){ error(s); }); }
    virtual LoggerStreamHelper fatal_stream() { return LoggerStreamHelper([this](const std::string& s){ fatal(s); }); }
    virtual ~Logger() = default;
};

#endif // LOGGING_UTILS_H

