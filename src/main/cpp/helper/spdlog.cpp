#include "helper/spdlog.h"

#include <chrono>
#include <memory>
#include <string>

#include <frc/Filesystem.h>
#include <spdlog/async.h>
#include <spdlog/async_logger.h>
#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

void handleSpdlogError(const std::string& msg)
{
    // Log when an error occurs within spdlog
    spdlog::error("*** SPDLOG ERROR ***: {}", msg);
}

void setupSpdlog()
{
    std::string operatingDirectory = frc::filesystem::GetOperatingDirectory();

    try
    {
        spdlog::init_thread_pool(256, 1);
        spdlog::flush_every(std::chrono::seconds(1));

        auto rotatingFileLogger = std::make_shared<spdlog::sinks::rotating_file_sink_mt>(
            operatingDirectory + "/logs/spdlog.log", 1024 * 1024, 10);
        auto stdoutLogger = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        auto logger = std::make_shared<spdlog::async_logger>(
            "spdlog",
            spdlog::sinks_init_list{
                rotatingFileLogger,
                stdoutLogger,
            },
            spdlog::thread_pool(),
            spdlog::async_overflow_policy::overrun_oldest);

        spdlog::set_default_logger(logger);
        spdlog::set_error_handler(handleSpdlogError);

        spdlog::info("spdlog initialized");
    }
    catch (spdlog::spdlog_ex e)
    {
        // If initialization fails, print the exception and fall back to the default stdout logger
        spdlog::error("*** SPDLOG EXCEPTION DURING INITIALIZATION: {} ***", e.what());
    }
}