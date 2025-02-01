#include <jse/jse.h>
#include "parse_jse.hpp"
#include <fstream>
#include <wmtk/utils/Logger.hpp>
namespace wmtk::applications::utils {
    nlohmann::json parse_jse(
            const nlohmann::json& spec,
            const std::filesystem::path& path) {

        std::ifstream ifs(path);
        nlohmann::json j = nlohmann::json::parse(ifs);

        jse::JSE spec_engine;
        bool r = spec_engine.verify_json(j, spec);
        if (!r) {
            wmtk::log_and_throw_error("{}", spec_engine.log2str());
        } else {
            j = spec_engine.inject_defaults(j, spec);
        }

        return j;
    }
}
