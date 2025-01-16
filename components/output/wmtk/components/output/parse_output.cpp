#include "parse_output.hpp"
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::output {
std::map<std::string, OutputOptions> parse_output(const nlohmann::json& js)
{
    std::map<std::string, OutputOptions> ret;
    try {
        ret = js;
        return ret;
    } catch (const std::exception& e) {
        wmtk::logger().debug(
            "Output failed to convert output to a vector, assuming just a single was required");
        try {
            ret[""] = js;
            return ret;
        } catch (const std::exception& e2) {
            throw e;
        }
    }
    return {};
}
} // namespace wmtk::components::output
