#pragma once

#include <nlohmann/json.hpp>

namespace wmtk {

// Applies a JSON-configured preallocation factor to a mesh, if present.
//
// The mesh storage (connectivity + attributes) is preallocated to
// max(floor, ceil(factor * live_count)) at init / consolidation; operations grab
// fresh slots from that headroom and fail (are retried later) once it is
// exhausted. The default (see TetMesh/TriMesh) suits the bundled integration
// tests; set "preallocation_factor" in the component's JSON to tune it per input
// (e.g. lower for pure-decimation runs, higher for aggressive refinement).
template <class Mesh>
inline void set_preallocation_factor_from_json(Mesh& mesh, const nlohmann::json& j)
{
    if (j.contains("preallocation_factor")) {
        mesh.set_preallocation_factor(j["preallocation_factor"].template get<double>());
    }
}

} // namespace wmtk
