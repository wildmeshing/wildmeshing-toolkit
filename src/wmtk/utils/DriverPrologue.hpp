#pragma once

#include <wmtk/utils/Logger.hpp>
#include <wmtk/utils/resolve_path.hpp>

#include <jse/jse.h>
#include <spdlog/common.h>
#include <nlohmann/json.hpp>

#include <filesystem>
#include <string>
#include <vector>

namespace wmtk::utils {

/**
 * @brief Verify a driver's json against its spec, inject the defaults, and set up the logger.
 *
 * The first thing tetwild, triwild and simwild each do, and they did it with 28 of 29
 * identical lines.
 *
 * @param[in,out] json_params  verified against `spec`, then replaced by the defaults-injected
 *                             version -- so the caller sees every optional key filled in
 * @param spec    the component's embedded jse spec
 * @return the resolved input directory, i.e. json_params["input_dir"]
 *
 * A key the spec does not have is refused, in all three: it is a typo or a removed option, and
 * ignoring it runs something the caller did not ask for -- an old skip_simplify: true would
 * simplify since simplify replaced it. Until 2026-11 only simwild refused one; tetwild and
 * triwild ignored it, warned only by wmtk_app and the Python bindings.
 */
inline std::filesystem::path verify_and_setup_logger(
    nlohmann::json& json_params,
    const nlohmann::json& spec)
{
    {
        jse::JSE spec_engine;
        spec_engine.strict = true;
        if (!spec_engine.verify_json(json_params, spec)) {
            log_and_throw_error(spec_engine.log2str());
        }
        json_params = spec_engine.inject_defaults(json_params, spec);
    }

    const std::filesystem::path root = json_params["input_dir"];

    {
        std::string log_file_name = json_params["log_file"];
        if (!log_file_name.empty()) {
            log_file_name = resolve_path(root, log_file_name).string();
            wmtk::set_file_logger(log_file_name);
            logger().flush_on(spdlog::level::info);
        }
    }

    return root;
}

} // namespace wmtk::utils
