#include "RegulatedIsotropicRemeshingOptions.hpp"

#include <nlohmann/json.hpp>

namespace wmtk::components::isotropic_remeshing {

namespace {
constexpr int64_t DEFAULT_MAX_COLLAPSES_PER_VERTEX = 1;
constexpr int64_t DEFAULT_MAX_POST_COLLAPSE_VALENCE = 9;
}

void RegulatedIsotropicRemeshingOptions::load_json(const nlohmann::json& js)
{
    IsotropicRemeshingOptions::load_json(js);

    max_collapses_per_vertex =
        js.value("max_collapses_per_vertex", DEFAULT_MAX_COLLAPSES_PER_VERTEX);
    limit_one_ring_collapses = js.value("limit_one_ring_collapses", true);
    enforce_max_valence = js.value("enforce_max_valence", true);
    max_post_collapse_valence =
        js.value("max_post_collapse_valence", DEFAULT_MAX_POST_COLLAPSE_VALENCE);
    count_skipped_as_failures = js.value("count_skipped_collapses_as_failures", false);
}

void RegulatedIsotropicRemeshingOptions::write_json(nlohmann::json& js) const
{
    IsotropicRemeshingOptions::write_json(js);

    js["max_collapses_per_vertex"] = max_collapses_per_vertex;
    js["limit_one_ring_collapses"] = limit_one_ring_collapses;
    js["enforce_max_valence"] = enforce_max_valence;
    js["max_post_collapse_valence"] = max_post_collapse_valence;
    js["count_skipped_collapses_as_failures"] = count_skipped_as_failures;
}

} // namespace wmtk::components::isotropic_remeshing

