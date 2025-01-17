#include "parse_output.hpp"
#include <nlohmann/json.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::output {
std::vector<std::pair<std::string, OutputOptions>> parse_output(const nlohmann::json& js)
{
    // try single position
    std::vector<std::pair<std::string, OutputOptions>> ret;

    // if it's an array we know we have multiple outputs
    if (js.is_array()) {
        for (const auto& val : js) {
            auto ret2 = parse_output(val);
            ret.insert(ret.end(), ret2.begin(), ret2.end());
        }
    } else {
        // try treating this input as a single object
        try {
            OutputOptions opt;
            opt = js;

            ret.emplace_back("", std::move(opt));
        } catch (const std::exception& e) {
            wmtk::logger().trace(
                "Output failed to parse as a single path, trying to parse as multiple");

            // try multi-object mode
            for (const auto& [key, value] : js.items()) {
                if (value.is_string()) {
                    throw std::runtime_error(fmt::format(
                        "Not allowed to have string output data as a value, input was {}, "
                        "offending "
                        "name/value were {}/{}",
                        js.dump(),
                        std::string(key),
                        std::string(value)));
                }
                if (value.is_array()) {
                    for (auto& val : value) {
                        ret.emplace_back(key, val);
                    }
                } else {
                    ret.emplace_back(key, value);
                }
            }
        }
    }
    return ret;
}
} // namespace wmtk::components::output
