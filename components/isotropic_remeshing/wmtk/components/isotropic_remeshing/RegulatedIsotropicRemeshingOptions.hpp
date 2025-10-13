#pragma once

#include "IsotropicRemeshingOptions.hpp"

#include <nlohmann/json_fwd.hpp>

namespace wmtk::components::isotropic_remeshing {

struct RegulatedIsotropicRemeshingOptions : public IsotropicRemeshingOptions
{
    /// Maximum number of collapses that can involve the same vertex within a single iteration.
    int64_t max_collapses_per_vertex = 1;
    /// Block the entire one-ring of a collapsed edge for the rest of the iteration.
    bool limit_one_ring_collapses = true;
    /// Enable rejecting collapses that would create vertices above the allowed valence.
    bool enforce_max_valence = true;
    /// Maximum valence allowed for the merged vertex created by a collapse.
    int64_t max_post_collapse_valence = 9;
    /// Count skipped collapses because of regulation as failures in statistics.
    bool count_skipped_as_failures = false;

    void load_json(const nlohmann::json& js);
    void write_json(nlohmann::json& js) const;
};

} // namespace wmtk::components::isotropic_remeshing

