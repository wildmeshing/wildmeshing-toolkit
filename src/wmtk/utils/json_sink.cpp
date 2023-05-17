#include "json_sink.h"
#include <spdlog/sinks/basic_file_sink.h>
namespace wmtk {
void set_json_format(spdlog::logger& logger, bool messages_are_json)
{
    if (messages_are_json) {
        const static std::string pattern = {
            "{\"time\": \"%Y-%m-%dT%H:%M:%S.%f%z\", \"epoch_secs\": %E, \"epoch_ms\": %e, "
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
    }
}
std::shared_ptr<spdlog::logger> make_json_file_logger(
    const std::string& name,
    const std::filesystem::path& path,
    bool messages_are_json)
{
    auto logger = spdlog::basic_logger_mt(name, path.string());
    set_json_format(*logger, messages_are_json);
    return logger;
}
} // namespace wmtk