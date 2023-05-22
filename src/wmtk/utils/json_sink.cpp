#include "json_sink.h"
#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
namespace wmtk {
namespace {
void set_json_format(spdlog::sinks::sink& logger, bool messages_are_json)
{
    // borrowed from github.com/mtao/balsa
    if (messages_are_json) {
        const static std::string pattern = {
            "{\"time\": \"%Y-%m-%dT%H:%M:%S.%f%z\", \"epoch_secs\": %E, \"epoch_ms\": \"%e\", "
            "\"name\": \"%n\", \"level\": \"%^%l%$\", \"process\": %P, \"thread\": %t, "
            "\"message\": %v}"};
        logger.set_pattern(pattern);
    } else {
        // https://github.com/gabime/spdlog/issues/1797
        const static std::string pattern = {
            "{\"time\": \"%Y-%m-%dT%H:%M:%S.%f%z\", \"epoch_secs\": %E, \"epoch_ms\": %e, "
            "\"name\": \"%n\", \"level\": \"%^%l%$\", \"process\": %P, \"thread\": %t, "
            "\"message\": \"%v\"}"};
        logger.set_pattern(pattern);
        logger.set_level(spdlog::level::trace);
    }
}

void set_json_format(spdlog::logger& logger, bool messages_are_json)
{
    for (const auto& sink_ptr : logger.sinks()) {
        set_json_format(*sink_ptr, messages_are_json);
    }
}
} // namespace
std::shared_ptr<spdlog::logger> make_json_file_logger(
    const std::string& name,
    const std::filesystem::path& path,
    bool messages_are_json,
    bool also_output_stdout)
{
    // create sinks
    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(path.string(), true);
    file_sink->set_level(spdlog::level::trace);
    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);


    std::vector<std::shared_ptr<spdlog::sinks::sink>> sinks;
    sinks.emplace_back(file_sink);
    if (also_output_stdout) {
        sinks.emplace_back(console_sink);
    }
    // dunno why make_shared didn't work for me today
    std::shared_ptr<spdlog::logger> js_logger;

    spdlog::init_thread_pool(8192, 1);

    js_logger.reset(new spdlog::async_logger(
        name,
        sinks.begin(),
        sinks.end(),
        spdlog::thread_pool(),
        spdlog::async_overflow_policy::block));


    set_json_format(*js_logger, messages_are_json);
    return js_logger;
}
} // namespace wmtk
