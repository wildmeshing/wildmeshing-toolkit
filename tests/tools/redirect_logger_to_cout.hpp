#pragma once
#include <spdlog/sinks/ostream_sink.h>
#include <wmtk/utils/Logger.hpp>

/**
 * @brief setting default and wmtk logger to std::cout
 *
 * This is just a helper for debugging tests.
 */
inline void redirect_logger_to_cout()
{
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(std::cout);
    auto ostream_logger = std::make_shared<spdlog::logger>("catch2_logger", ostream_sink);
    ostream_logger->set_level(spdlog::level::debug);
    spdlog::set_default_logger(ostream_logger);
    wmtk::set_logger(ostream_logger);
}