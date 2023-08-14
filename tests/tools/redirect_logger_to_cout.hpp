#pragma once
#include <spdlog/sinks/ostream_sink.h>
#include <wmtk/utils/Logger.hpp>

/**
 * @brief setting default and wmtk logger to std::cout
 *
 * This is just a helper for debugging tests. In Visual Studio, spdlog is for some reason not
 * forwarded to std::cout by default. This function should only be used for debugging and it should
 * never appear in any commit.
 *
 * References:
 * https://stackoverflow.com/questions/66473052/how-can-i-read-spdlog-output-in-a-google-test
 * https://github.com/gabime/spdlog/issues/1859
 */
inline void redirect_logger_to_cout()
{
    auto ostream_sink = std::make_shared<spdlog::sinks::ostream_sink_st>(std::cout);
    auto ostream_logger = std::make_shared<spdlog::logger>("catch2_logger", ostream_sink);
    ostream_logger->set_level(spdlog::level::debug);
    spdlog::set_default_logger(ostream_logger);
    wmtk::set_logger(ostream_logger);
}