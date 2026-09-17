#include "polyfem_ops.hpp"

#include <jse/jse.h>
#include <polyfem_ops_spec.hpp>
#include <wmtk/utils/Logger.hpp>

namespace wmtk::components::polyfem_ops {

void polyfem_ops(nlohmann::json json_params)
{
    // Stage 1 of the port: the component exists, links polyfem, and validates the dispatcher keys.
    // The operations arrive in stage 2; each validates its own parameters against the pysimwild
    // spec.json that defines them, so only the dispatcher keys are checked here.
    const auto spec = jse::embed::wmtk_polyfem_ops_spec::polyfem_ops_spec::spec();
    jse::JSE spec_engine;
    spec_engine.strict = true;

    nlohmann::json dispatch = nlohmann::json::object();
    for (const char* key : {"application", "input", "output", "operation"}) {
        if (json_params.contains(key)) {
            dispatch[key] = json_params[key];
        }
    }
    if (!spec_engine.verify_json(dispatch, spec)) {
        log_and_throw_error(spec_engine.log2str());
    }

    const std::string operation = dispatch["operation"].get<std::string>();
    log_and_throw_error(
        "polyfem_ops: operation '" + operation +
        "' is not ported yet (stage 2 of the port plan); use the Python engine");
}

} // namespace wmtk::components::polyfem_ops
