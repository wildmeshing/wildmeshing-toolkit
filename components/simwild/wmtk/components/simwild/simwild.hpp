#pragma once

#include <nlohmann/json.hpp>


namespace wmtk::components::simwild {

void simwild(nlohmann::json json_params);

/**
 * @brief The embedded simwild spec, with the keys the caller put in `/amips_weights` named in it.
 *
 * `/amips_weights` is a free-form object -- its keys are the caller's own tag or group names --
 * and jse cannot express that. Its pointers match exactly, so the `/amips_weights/*` rule is never
 * consulted for a concrete key, and in strict mode an object rule with no `optional` list refuses
 * every non-empty dict (jse.cpp, verify_rule_object via find_extra_keys). The equivalent is to NAME
 * the keys: the ones the caller actually passed become the `optional` list and each gets its own
 * copy of the `*` rule, with the default "skip" that jse requires of an optional key and that
 * leaves an absent key absent. This is the one part of the spec that needs the parameters and not
 * just the spec, which is why it is built here rather than written into simwild_spec.json. Built
 * in every configuration, polyfem or not, so that a job carrying /amips_weights reaches the
 * operation's own error rather than a validation failure.
 *
 * With no `/amips_weights` in `json_params` the spec is returned unchanged.
 */
nlohmann::json simwild_spec_for(const nlohmann::json& json_params);

} // namespace wmtk::components::simwild
