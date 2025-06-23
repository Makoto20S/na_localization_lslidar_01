#ifndef _LOG_H_
#define _LOG_H_



#include "spdlog/spdlog.h"
#include "spdlog/async.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/sinks/rotating_file_sink.h"

namespace LOG{
    class LOGGER final
    {
    private:
        LOGGER() {};
    public:
        static void setLogger(std::shared_ptr<spdlog::async_logger> logger_input){
            my_logger_ = std::move(logger_input);
        }

        static std::shared_ptr<spdlog::async_logger> getLog(){
            return my_logger_;
        }
    
    private:
        static std::shared_ptr<spdlog::async_logger> my_logger_;      
    };
}

#define NLOG_TRACE(...) SPDLOG_LOGGER_CALL(LOG::LOGGER::getLog(), spdlog::level::trace, ##__VA_ARGS__)
#define NLOG_DEBUG(...) SPDLOG_LOGGER_CALL(LOG::LOGGER::getLog(), spdlog::level::debug, ##__VA_ARGS__)
#define NLOG_INFO(...) SPDLOG_LOGGER_CALL(LOG::LOGGER::getLog(), spdlog::level::info, ##__VA_ARGS__)
#define NLOG_WARN(...) SPDLOG_LOGGER_CALL(LOG::LOGGER::getLog(), spdlog::level::warn, ##__VA_ARGS__)
#define NLOG_ERROR(...) SPDLOG_LOGGER_CALL(LOG::LOGGER::getLog(), spdlog::level::err, ##__VA_ARGS__)
#define NLOG_CRITICAL(...) SPDLOG_LOGGER_CALL(LOG::LOGGER::getLog(), spdlog::level::critical, ##__VA_ARGS__)

#endif