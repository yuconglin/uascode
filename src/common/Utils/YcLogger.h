#pragma once

#include <log4cxx/logger.h>

namespace Utils{

struct LogConfigurator
{
    LogConfigurator(const std::string& fname, const std::string& msg);
};

typedef log4cxx::LoggerPtr LoggerPtr;
//typedef log4cxx::NDC NDC;

inline LoggerPtr getLogger(const char* loggerName)
{
    return log4cxx::Logger::getLogger(loggerName);
}

log4cxx::LevelPtr getStartup();

#define LL_TRACE log4cxx::Level::getTrace()
#define LL_DEBUG log4cxx::Level::getDebug()
#define LL_INFO log4cxx::Level::getInfo()
#define LL_WARN log4cxx::Level::getWarn()
#define LL_ERROR log4cxx::Level::getError()
#define LL_FATAL log4cxx::Level::getFatal()
#define LL_STARTUP sri::log::getStartup()

#define UASLOG(logger, level, message) { \
    if (logger->isEnabledFor(level)) { \
        ::log4cxx::helpers::MessageBuffer oss_; \
        logger->forcedLog(level, \
                          oss_.str(oss_ << message), LOG4CXX_LOCATION); \
    } }

}
